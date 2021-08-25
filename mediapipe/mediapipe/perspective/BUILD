licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "perspective",
    srcs = ["perspective.cc"],
    deps = [
        "//mediapipe/graphs/face_mesh:desktop_live_gpu_calculators",
        "//mediapipe/graphs/face_effect:face_effect_gpu_deps",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/gpu:gl_calculator_helper",
        "//mediapipe/gpu:gpu_buffer",
        "//mediapipe/gpu:gpu_shared_data_internal",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    data = [
        "//mediapipe/graphs/face_effect/data:facepaint.pngblob",
        "//mediapipe/modules/face_detection:face_detection_front.tflite",
        "//mediapipe/modules/face_geometry/data:geometry_pipeline_metadata.binarypb",
        "//mediapipe/modules/face_geometry/data:geometry_pipeline_metadata_detection.binarypb",
        "//mediapipe/modules/face_geometry/data:geometry_pipeline_metadata_landmarks.binarypb",
        "//mediapipe/modules/face_landmark:face_landmark.tflite",
    ]
)
