import os
from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy


class EmbodiedDataSlamConan(ConanFile):
    name = "embodied_data_slam"
    version = "2.0.0"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "build_station": [True, False],
        "build_backend_deps": [True, False],
    }
    default_options = {
        "build_station": True,
        "build_backend_deps": True,
        # Static linking for self-contained native executables on Windows/Linux/macOS
        "opencv/*:shared": False,
        "opencv/*:dnn": False,
        "opencv/*:gapi": False,
        "opencv/*:ml": False,
        "opencv/*:stitching": False,
        "opencv/*:with_openexr": False,
        "opencv/*:with_tiff": False,
        "opencv/*:with_webp": False,
        "opencv/*:with_jpeg2000": False,
        "opencv/*:with_quirc": False,
        "opencv/*:with_eigen": False,
        "opencv/*:with_ffmpeg": True,
        # Dynamic-link pure-LGPL FFmpeg (with built-in MKV/MP4/H264/HEVC/MJPEG/FFV1 decoders)
        # so commercial closed-source distribution complies with LGPLv2.1+ while avoiding
        # GPL/external encoders (libx265 also fails on MSVC 195).
        "ffmpeg/*:shared": True,
        "ffmpeg/*:postproc": False,
        "ffmpeg/*:avdevice": False,
        "ffmpeg/*:with_programs": False,
        "ffmpeg/*:with_libx264": False,
        "ffmpeg/*:with_libx265": False,
        "ffmpeg/*:with_libvpx": False,
        "ffmpeg/*:with_libmp3lame": False,
        "ffmpeg/*:with_libfdk_aac": False,
        "ffmpeg/*:with_libwebp": False,
        "ffmpeg/*:with_opus": False,
        "ffmpeg/*:with_vorbis": False,
        "ffmpeg/*:with_libaom": False,
        "ffmpeg/*:with_libdav1d": False,
        "ffmpeg/*:with_openh264": False,
        "ffmpeg/*:with_openjpeg": False,
        "ffmpeg/*:with_freetype": False,
        "ffmpeg/*:with_libiconv": False,
        "ffmpeg/*:with_bzip2": False,
        "ffmpeg/*:with_lzma": False,
        "ffmpeg/*:with_ssl": False,
        "boost/*:shared": False,
        "boost/*:without_atomic": True,
        "boost/*:without_charconv": True,
        "boost/*:without_chrono": True,
        "boost/*:without_cobalt": True,
        "boost/*:without_container": True,
        "boost/*:without_context": True,
        "boost/*:without_contract": True,
        "boost/*:without_coroutine": True,
        "boost/*:without_date_time": True,
        "boost/*:without_exception": True,
        "boost/*:without_fiber": True,
        "boost/*:without_filesystem": True,
        "boost/*:without_graph": True,
        "boost/*:without_graph_parallel": True,
        "boost/*:without_iostreams": True,
        "boost/*:without_json": True,
        "boost/*:without_locale": True,
        "boost/*:without_log": True,
        "boost/*:without_math": True,
        "boost/*:without_mpi": True,
        "boost/*:without_nowide": True,
        "boost/*:without_program_options": True,
        "boost/*:without_python": True,
        "boost/*:without_random": True,
        "boost/*:without_regex": True,
        "boost/*:without_stacktrace": True,
        "boost/*:without_system": True,
        "boost/*:without_test": True,
        "boost/*:without_thread": True,
        "boost/*:without_timer": True,
        "boost/*:without_type_erasure": True,
        "boost/*:without_url": True,
        "boost/*:without_wave": True,
        "openssl/*:shared": False,
        "glew/*:shared": False,
        "glfw/*:shared": False,
        "imgui/*:shared": False,
    }

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("opencv/4.10.0")
        if self.options.build_backend_deps:
            self.requires("boost/1.86.0")
            self.requires("openssl/3.3.2")
            self.requires("glew/2.2.0")
        if self.options.build_station:
            self.requires("imgui/1.91.5-docking")
            self.requires("glfw/3.4")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["EDS_BUILD_STATION"] = bool(self.options.build_station)
        tc.generate()

        # Collect any shared runtime libraries (e.g. LGPL FFmpeg DLL/SO/DYLIB) for staging
        runtime_dst = os.path.join(self.generators_folder, "runtime_libs")
        for dep in self.dependencies.host.values():
            for bindir in dep.cpp_info.bindirs:
                if os.path.isdir(bindir):
                    copy(self, "*.dll", src=bindir, dst=runtime_dst)
            for libdir in dep.cpp_info.libdirs:
                if os.path.isdir(libdir):
                    copy(self, "*.so*", src=libdir, dst=runtime_dst)
                    copy(self, "*.dylib*", src=libdir, dst=runtime_dst)

        if self.options.build_station:
            imgui_dep = self.dependencies["imgui"]
            bindings_src = os.path.join(imgui_dep.package_folder, "res", "bindings")
            if not os.path.isdir(bindings_src) and imgui_dep.cpp_info.srcdirs:
                bindings_src = imgui_dep.cpp_info.srcdirs[0]
            bindings_dst = os.path.join(self.generators_folder, "imgui_bindings")
            for pattern in (
                "imgui_impl_glfw.h",
                "imgui_impl_glfw.cpp",
                "imgui_impl_opengl3.h",
                "imgui_impl_opengl3.cpp",
                "imgui_impl_opengl3_loader.h",
            ):
                copy(self, pattern, src=bindings_src, dst=bindings_dst)
