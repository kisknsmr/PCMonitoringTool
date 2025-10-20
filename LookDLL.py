import os, ctypes

dllpath = r"E:\OutOfOneDrive\Programs\01.REPOS\PycharmProjects\HelloGalaxy\pcmonitor.dll"
dll = ctypes.CDLL(dllpath)

# 1) 文字列返すバージョン関数（あれば超便利）
dll.pm_version.restype = ctypes.c_char_p
print("pm_version:", dll.pm_version().decode())

# 2) FastLog ヘッダ→1行追記の最小パス
logdir = r"E:\OutOfOneDrive\Programs\01.REPOS\PycharmProjects\HelloGalaxy\logs"
os.makedirs(logdir, exist_ok=True)
fast_csv = os.path.join(logdir, "fast_test.csv").encode()

dll.pm_log_fast_write_header.argtypes = [ctypes.c_char_p]
dll.pm_log_fast_write_header.restype = ctypes.c_int
print("write_header:", dll.pm_log_fast_write_header(fast_csv))

dll.pm_log_system_fast.argtypes = [ctypes.c_char_p]
dll.pm_log_system_fast.restype = ctypes.c_int
print("log_fast:", dll.pm_log_system_fast(fast_csv))
