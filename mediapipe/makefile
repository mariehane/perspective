run: perspective
	export GLOG_logtostderr=1; cd bazel-bin; ./mediapipe/perspective/perspective

makefile_dir=$(shell pwd)

perspective: mediapipe/perspective/*
	bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11 mediapipe/perspective:perspective
	echo 
	# setup data files
	cd bazel-bin/mediapipe/perspective; \
		cp -iu $(makefile_dir)/mediapipe/perspective/perspective_graph_gpu.pbtxt .; \
		cp -iur $(makefile_dir)/mediapipe/perspective/data/ .
	cd bazel-bin/mediapipe/modules/face_detection/; \
		cp -iu $(makefile_dir)/mediapipe/modules/face_detection/face_detection_front.tflite .; \
		cp -iu $(makefile_dir)/mediapipe/modules/face_detection/face_detection_short_range.tflite .
	cd bazel-bin/mediapipe/modules/face_landmark/; \
		cp -iu $(makefile_dir)/mediapipe/modules/face_landmark/face_landmark.tflite .


run_test: test
	export GLOG_logtostderr=1; cd bazel-bin; ./mediapipe/testing/test

test:
	bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11 mediapipe/testing:test
#	bazel build -c opt mediapipe/testing:test
#	bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/testing:test

run_face_mesh_gpu: face_mesh_gpu
	./bazel-bin/mediapipe/examples/desktop/face_mesh/face_mesh_gpu --calculator_graph_config_file mediapipe/graphs/face_mesh/face_mesh_desktop_live_gpu.pbtxt

face_mesh_gpu:
	bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS --copt -DEGL_NO_X11 mediapipe/examples/desktop/face_mesh:face_mesh_gpu
