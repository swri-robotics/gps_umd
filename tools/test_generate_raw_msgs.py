#!/usr/bin/env python3
"""Unit tests for generate_raw_msgs.py.

These feed known gps.h fragments to the parser rather than the real header, so
they pin the behaviour that matters without needing a GPSd checkout. Run with:

    python3 -m unittest discover -s tools -p 'test_*.py'
"""

import importlib.util
import os
import unittest

_SPEC = importlib.util.spec_from_file_location(
    "generate_raw_msgs",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "generate_raw_msgs.py"))
gen = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen)


class SnakeCase(unittest.TestCase):
    def test_gpsd_member_names(self):
        # GPSd mixes conventions freely; all of these are real member names.
        self.assertEqual(gen.snake_case("PRN"), "prn")
        self.assertEqual(gen.snake_case("altHAE"), "alt_hae")
        self.assertEqual(gen.snake_case("altMSL"), "alt_msl")
        self.assertEqual(gen.snake_case("relPosN"), "rel_pos_n")
        self.assertEqual(gen.snake_case("errEllipseOrient"), "err_ellipse_orient")
        self.assertEqual(gen.snake_case("qualityInd"), "quality_ind")
        self.assertEqual(gen.snake_case("dgps_age"), "dgps_age")
        self.assertEqual(gen.snake_case("NED"), "ned")

    def test_matches_rosidl_header_names(self):
        # Verified against the headers rosidl actually emitted; the acronym
        # split is the part that is easy to get wrong (not "gpsdbaseline").
        self.assertEqual(gen.ros_header_name("GPSDBaseline16v1"), "gpsd_baseline16v1")
        self.assertEqual(gen.ros_header_name("GPSDRaw16v1"), "gpsd_raw16v1")
        self.assertEqual(gen.ros_header_name("GPSDFixEcef16v1"), "gpsd_fix_ecef16v1")
        self.assertEqual(gen.ros_header_name("GPSDFixNED16v1"), "gpsd_fix_ned16v1")


class MessageNames(unittest.TestCase):
    def test_struct_tag_to_message_stem(self):
        self.assertEqual(gen.message_base_name("gps_data_t"), "GPSDRaw")
        self.assertEqual(gen.message_base_name("gps_fix_t"), "GPSDFix")
        self.assertEqual(gen.message_base_name("satellite_t"), "GPSDSatellite")
        self.assertEqual(gen.message_base_name("dop_t"), "GPSDDop")
        self.assertEqual(gen.message_base_name("baseline_t"), "GPSDBaseline")

    def test_versioned(self):
        self.assertEqual(gen.versioned("GPSDRaw", (16, 1)), "GPSDRaw16v1")
        self.assertEqual(gen.versioned("GPSDRaw", (9, 0)), "GPSDRaw9v0")


class SplitMembers(unittest.TestCase):
    def parse(self, body):
        return {m.name: m for m in gen.split_members(gen.resolve_conditionals(body))}

    def test_scalars_and_multi_declarators(self):
        members = self.parse("double x, y, z; int mode; unsigned char gnssid;")
        self.assertEqual(set(members), {"x", "y", "z", "mode", "gnssid"})
        self.assertEqual(members["z"].ctype, "double")
        # The multi-word type must not be split into "unsigned" + "char gnssid".
        self.assertEqual(members["gnssid"].ctype, "unsigned char")

    def test_arrays(self):
        members = self.parse("char datum[40]; struct satellite_t skyview[MAXCHANNELS];")
        self.assertEqual(members["datum"].array, "40")
        self.assertEqual(members["skyview"].array, "MAXCHANNELS")
        self.assertEqual(members["skyview"].ctype, "struct satellite_t")

    def test_named_struct_member(self):
        members = self.parse("struct gps_fix_t fix;")
        self.assertEqual(members["fix"].ctype, "struct gps_fix_t")
        self.assertIsNone(members["fix"].anon_body)

    def test_inline_anonymous_struct_with_name(self):
        members = self.parse("struct { double x, y, z; } ecef;")
        self.assertIn("ecef", members)
        self.assertIsNotNone(members["ecef"].anon_body)
        inner = {m.name for m in gen.split_members(members["ecef"].anon_body)}
        self.assertEqual(inner, {"x", "y", "z"})

    def test_anonymous_union_members_are_spliced_into_parent(self):
        # C11 6.7.2.1: an anonymous union injects its members into the
        # enclosing scope. gps_data_t relies on this for the report arms, and
        # it is what lets the scope filters and the AIS exclusion match by name.
        members = self.parse(
            "struct gst_t gst; union { struct rtcm2_t rtcm2; "
            "struct ais_t ais; char error[256]; }; int leap_seconds;")
        self.assertEqual(set(members),
                         {"gst", "rtcm2", "ais", "error", "leap_seconds"})

    def test_function_pointer(self):
        members = self.parse("void (*update_fd)(int fd, bool open);")
        self.assertEqual(members["update_fd"].ctype, "function_pointer")

    def test_defines_inside_body_are_not_members(self):
        members = self.parse("int mode;\n#define MODE_2D 2\n#define MODE_3D 3\ndouble ept;")
        self.assertEqual(set(members), {"mode", "ept"})


class Conditionals(unittest.TestCase):
    def test_use_qt_is_treated_as_undefined(self):
        body = ("#ifndef USE_QT\n  int gps_fd;\n#else\n  void* gps_fd;\n#endif\n"
                "  double ept;")
        members = {m.name: m for m in gen.split_members(gen.resolve_conditionals(body))}
        self.assertEqual(members["gps_fd"].ctype, "int")

    def test_multiline_define_is_dropped_whole(self):
        # UNION_SET is a line-continued #define inside gps_data_t; dropping only
        # its first line would leave the rest looking like struct members.
        body = ("#define UNION_SET (AIS_SET|ERROR_SET| \\\n"
                "                   RTCM2_SET|TOFF_SET)\n"
                "  double ept;")
        members = gen.split_members(gen.resolve_conditionals(body))
        self.assertEqual([m.name for m in members], ["ept"])

    def test_unknown_conditional_raises_rather_than_guessing(self):
        with self.assertRaises(SystemExit):
            gen.resolve_conditionals("#ifdef SOMETHING_ELSE\n int x;\n#endif")


class MaskConstants(unittest.TestCase):
    SRC = ("#define ONLINE_SET (1llu<<1)\n"
           "#define LATLON_SET (1llu<<4)\n"
           "#define AIS_SET (1llu<<24)\n"
           "#define ERROR_SET (1llu<<31)\n"
           "#define UNION_SET (AIS_SET|ERROR_SET)\n"
           "#define SET_HIGH_BIT 46\n")

    def test_names_are_flipped_to_avoid_gps_h_macros(self):
        constants = dict(gen.mask_constants(self.SRC))
        # rosidl emits constants as static constexpr members and gps.h defines
        # LATLON_SET etc. as global macros, so the GPSd spelling is unusable.
        self.assertIn("SET_LATLON", constants)
        self.assertNotIn("LATLON_SET", constants)
        self.assertEqual(constants["SET_ONLINE"], str(1 << 1))
        self.assertEqual(constants["SET_LATLON"], str(1 << 4))

    def test_union_is_resolved_and_keeps_ais(self):
        constants = dict(gen.mask_constants(self.SRC))
        self.assertEqual(constants["SET_UNION"], str((1 << 24) | (1 << 31)))
        # AIS is not published, but the constant must still mean what
        # gps.h says, so consumers can detect an omitted AIS report.
        self.assertIn("SET_AIS", constants)

    def test_high_bit_is_renamed_away_from_the_gps_h_macro(self):
        # gps.h defines SET_HIGH_BIT itself, so emitting that name produced a
        # message constant the preprocessor destroyed. Carried through under a
        # name gps.h does not use.
        constants = dict(gen.mask_constants(self.SRC))
        self.assertEqual(constants["SET_HIGHEST_BIT"], "46")
        self.assertNotIn("SET_HIGH_BIT", constants)

    def test_macro_collisions_are_rejected(self):
        with self.assertRaises(SystemExit):
            gen.assert_no_macro_collisions(
                [("SET_HIGH_BIT", "46")], "#define SET_HIGH_BIT 46\n", "test-rev")
        # A name gps.h does not define is fine.
        gen.assert_no_macro_collisions(
            [("SET_HIGHEST_BIT", "46")], "#define SET_HIGH_BIT 46\n", "test-rev")

    def test_union_naming_an_unknown_bit_raises(self):
        with self.assertRaises(SystemExit):
            gen.mask_constants("#define UNION_SET (NOSUCH_SET)\n")


class Manifest(unittest.TestCase):
    def test_ten_pairs_and_no_api_15(self):
        self.assertEqual(len(gen.REFERENCE_REVS), 10)
        self.assertEqual(set(gen.REFERENCE_REVS), set(gen.EXPECTED_MAXCHANNELS))
        # GPSD_API_MAJOR_VERSION jumped 14 -> 16; 15 was never a real value.
        self.assertFalse([p for p in gen.REFERENCE_REVS if p[0] == 15])

    def test_maxchannels_only_ever_140_or_184(self):
        # The gps.h changelog claims 185 and 230; no release shipped either.
        self.assertEqual(set(gen.EXPECTED_MAXCHANNELS.values()), {140, 184})

    def test_ais_is_excluded(self):
        self.assertIn("ais", gen.EXCLUDED_MEMBERS)


class TypeMapping(unittest.TestCase):
    def test_unknown_type_is_a_hard_error(self):
        # Silently dropping an unmapped type would lose data without a trace.
        model = gen.Model((16, 1), "")
        with self.assertRaises(SystemExit):
            gen.add_field(model, [], gen.Member(name="x", ctype="weird_t"),
                          parent="gps_data_t")

    def test_colliding_ros_names_are_a_hard_error(self):
        model = gen.Model((16, 1), "")
        fields = []
        gen.add_field(model, fields, gen.Member(name="altHAE", ctype="double"),
                      parent="gps_fix_t")
        with self.assertRaises(SystemExit):
            gen.add_field(model, fields, gen.Member(name="alt_hae", ctype="double"),
                          parent="gps_fix_t")


if __name__ == "__main__":
    unittest.main()
