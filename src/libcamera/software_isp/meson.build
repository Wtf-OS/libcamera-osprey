# SPDX-License-Identifier: CC0-1.0

softisp_enabled = pipelines.contains('simple')
summary({'SoftISP support' : softisp_enabled},
        bool_yn : true,
        section : 'Configuration')

if not softisp_enabled
    subdir_done()
endif

libegl = dependency('egl', required : get_option('softisp-gpu'))
libglesv2 = dependency('glesv2', required : get_option('softisp-gpu'))
mesa_works = cc.check_header('EGL/egl.h',
                             required : get_option('softisp-gpu'),
                             dependencies : libegl)

summary({'SoftISP GPU acceleration' : mesa_works},
        bool_yn : true,
        section : 'Configuration')

libcamera_internal_sources += files([
    'benchmark.cpp',
    'debayer.cpp',
    'debayer_cpu.cpp',
    'software_isp.cpp',
    'swstats_cpu.cpp',
])

if mesa_works
    config_h.set('HAVE_DEBAYER_EGL', 1)
    libcamera_internal_sources += files([
        '../egl.cpp',
        'debayer_egl.cpp',
    ])
    libcamera_deps += [
        libegl,
        libglesv2,
    ]
endif

libcamera_deps += dependency('libyuv')

raw64_compiler = get_option('softisp-raw64-compiler')
if raw64_compiler != ''
    raw64_cpp = find_program(raw64_compiler, required : true)
    raw64_worker = custom_target('raw64_worker',
        input : files('raw64/raw-abi-worker.cpp'),
        depend_files : files('raw64/raw-abi-protocol.h', 'raw64/raw-abi-render.h',
                             'raw64/raw-abi-engine-generated.h', 'raw64/raw-abi-colour-generated.h'),
        output : 'raw64_worker',
        command : [raw64_cpp, '-std=c++20', '-O3', '-Wall', '-Wextra', '-Werror',
                   '-pthread', '-static', '@INPUT@', '-o', '@OUTPUT@'],
        install : true,
        install_dir : get_option('libexecdir') / 'libcamera')
endif
