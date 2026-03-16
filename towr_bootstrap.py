import os
import sys


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
                os.environ["PATH"] = dll_dir + os.pathsep + os.environ.get("PATH", "")


def load_towr():
    configure_towr_environment()
    import towr_cpp as towr

    return towr