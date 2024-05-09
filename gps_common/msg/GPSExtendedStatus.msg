# Extended Measurement status to use for GPSStatus.msg
int16 STATUS_NO_FIX=-1    # Unable to fix position
int16 STATUS_FIX=0        # Normal fix
int16 STATUS_SBAS_FIX=1   # Fixed using a satellite-based augmentation system
int16 STATUS_GBAS_FIX=2   #          or a ground-based augmentation system
int16 STATUS_DGPS_FIX=18  # Fixed with DGPS
int16 STATUS_RTK_FIX=19   # Real-Time Kinematic, fixed integers
int16 STATUS_RTK_FLOAT=20 # Real-Time Kinematic, float integers
int16 STATUS_WAAS_FIX=33  # Fixed with WAAS
