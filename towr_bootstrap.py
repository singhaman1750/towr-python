import os
import sys
import ctypes


def _preload_linux_cpp_runtime():
    if os.name == "nt":
        return

    # Prefer the active conda env runtime to satisfy newer CXXABI requirements
    # from IPOPT dependencies (e.g. libspral).
    # 1) Prefer the active interpreter prefix.
    #    This works even without `conda activate`.
    py_prefix = os.path.dirname(os.path.dirname(sys.executable))
    candidate_lib_dirs = []
    candidate_lib_dirs.append(os.path.join(py_prefix, "lib"))

    # 2) Then prefer active conda env, if exported
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        candidate_lib_dirs.append(os.path.join(conda_prefix, "lib"))

    # 3) Finally consider base conda install.
    conda_exe = os.environ.get("CONDA_EXE")
    if conda_exe:
        base_prefix = os.path.dirname(os.path.dirname(conda_exe))
        candidate_lib_dirs.append(os.path.join(base_prefix, "lib"))

    # Keep order while removing duplicates.
    candidate_lib_dirs = list(dict.fromkeys(candidate_lib_dirs))

    loaded = False
    for lib_dir in candidate_lib_dirs:
        libstdcpp = os.path.join(lib_dir, "libstdc++.so.6")
        libgcc = os.path.join(lib_dir, "libgcc_s.so.1")
        if os.path.exists(libstdcpp):
            try:
                ctypes.CDLL(libstdcpp, mode=ctypes.RTLD_GLOBAL)
                if os.path.exists(libgcc):
                    ctypes.CDLL(libgcc, mode=ctypes.RTLD_GLOBAL)
                loaded = True
                break
            except OSError:
                pass

    # Keep library dir discoverable for transitive solver dependencies.
    if loaded:
        ld_path = os.environ.get("LD_LIBRARY_PATH", "")
        if lib_dir not in ld_path.split(os.pathsep):
            os.environ["LD_LIBRARY_PATH"] = (
                lib_dir if not ld_path else lib_dir + os.pathsep + ld_path
            )


def configure_towr_environment():
    repo_dir = os.path.dirname(__file__)
    cmake_dir = os.path.join(repo_dir, "towr")
    candidate_dirs = [
        os.path.join(cmake_dir, "build"),
        os.path.join(cmake_dir, "build", "Release"),
        os.path.join(cmake_dir, "build", "Debug"),
    ]

    for candidate in candidate_dirs:
        if os.path.exists(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)

    _preload_linux_cpp_runtime()

    if os.name == "nt":
        dll_dirs = [
            os.path.join(sys.prefix, "Library", "bin"),
            os.path.join(cmake_dir, "build", "Release"),
            os.path.join(cmake_dir, "build", "Debug"),
            os.path.join(repo_dir, "ifopt", "install", "bin"),
        ]
        for dll_dir in dll_dirs:
            if os.path.exists(dll_dir):
                if hasattr(os, "add_dll_directory"):
                    os.add_dll_directory(dll_dir)
                os.environ["PATH"] = (
                    dll_dir + os.pathsep + os.environ.get("PATH", "")
                )


def load_towr():
    configure_towr_environment()
    import towr_cpp as towr

    return towr
