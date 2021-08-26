// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
// This example requires a linux computer and a GPU with EGL support drivers.
#include <cstdlib>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";
//constexpr char FACEMESH_GRAPH_CONFIG_FILE[] = "mediapipe/graphs/face_mesh/face_mesh_desktop_live_gpu.pbtxt";
//constexpr char FACEMESH_GRAPH_CONFIG_FILE[] = "mediapipe/graphs/face_effect/face_effect_gpu.pbtxt";
constexpr char FACEMESH_GRAPH_CONFIG_FILE[] = "mediapipe/perspective/perspective_graph_gpu.pbtxt";

//constexpr char kUseFaceDetectionSidePacket[] = "use_face_detection_input_source";
constexpr char kSelectedFilterInputStream[] = "selected_effect_id";

constexpr int kMinAge = 14;
constexpr int kMaxAge = 80;

constexpr int kMinCigarettes = 0;
constexpr int kMaxCigarettes = 50;

constexpr int kKeyESC = 28;
constexpr int kKeyA = 97;
constexpr int kKeyQ = 113;
constexpr int kKeyS = 115;
constexpr int kKeyW = 119;
constexpr int kKeyX = 120;

constexpr char kComma[] = ",";
constexpr int kArduinoBufferSize = 1024;

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

// converts an age and number of cigarettes to a filter index
//              Age
//          … 30 … 60 …
//       0
//       ⋮  0   1   2
//       3
// Cigs  ⋮  3   4   5
//       6
//       ⋮  6   7   8 
//
int getFilterIndex(int age, int cigarettes) {
  int index = 0;
  if (age >= 30 && age < 60) {
    index += 1;
  } else if (age >= 60) {
    index += 2;
  }
  if (cigarettes >= 3 && cigarettes < 6) {
    index += 3;
  } else if (cigarettes >= 6) {
    index += 6;
  }
  return index;
}

// open a connection to the arduino and get a FILE* to read from it
FILE* getArduino() {
    int arduino = open( "/dev/ttyACM0", O_RDWR | O_NONBLOCK | O_NDELAY );
    if (arduino < 0)
    {
        //cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << endl;
        return NULL;
    }

    /* *** Configure Port *** */
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if ( tcgetattr ( arduino, &tty ) != 0 )
    {
        //out << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return NULL;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, B9600);
    cfsetispeed (&tty, B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;   // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;  // no flow control
    tty.c_lflag     =   0;         // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;         // no remapping, no delays
    tty.c_cc[VMIN]  =   0;         // read doesn't block
    tty.c_cc[VTIME] =   5;         // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;          // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag     &=  ~OPOST;                  // make raw

    /* Flush Port, then applies attributes */
    tcflush( arduino, TCIFLUSH );
    if ( tcsetattr ( arduino, TCSANOW, &tty ) != 0)
    {
        //cout << "Error " << errno << " from tcsetattr" << endl;
        return NULL;
    }

    /* Get FILE* */
    FILE* arduinofp = fdopen(arduino, "r");
    return arduinofp;
}

typedef struct {
  int val1;
  int val2;
} SliderValues;

SliderValues* getSliderValues(FILE* arduino) { 
  /* Allocate memory for read buffer */
  char buf [kArduinoBufferSize];
  memset (&buf, '\0', sizeof(buf));

  if (fgets( buf, sizeof(buf), arduino ) == NULL) {
    return NULL;
  }
  
  char *str1 = strtok(buf, kComma);
  if (str1 == NULL) {
    return NULL;
  }

  char *str2 = strtok(NULL, kComma);
  if (str2 == NULL) {
    return NULL;
  }

  SliderValues* result = new SliderValues();
  result->val1 = atoi(str1);
  result->val2 = atoi(str2);
  return result;
}

absl::Status RunMPPGraph() {
  LOG(INFO) << "Connect to Arduino.";
  FILE* arduino = getArduino();
  if (arduino == NULL) {
    LOG(ERROR) << "Could not connect to Arduino!";
    //exit(1);
  }

  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      FACEMESH_GRAPH_CONFIG_FILE,
      //absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the GPU.";
  ASSIGN_OR_RETURN(auto gpu_resources, mediapipe::GpuResources::Create());
  MP_RETURN_IF_ERROR(graph.SetGpuResources(std::move(gpu_resources)));
  mediapipe::GlCalculatorHelper gpu_helper;
  gpu_helper.InitializeForTest(graph.GetGpuResources().get());

  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  if (load_video) {
    capture.open(absl::GetFlag(FLAGS_input_video_path));
  } else {
    capture.open(0);
  }
  RET_CHECK(capture.isOpened());

  cv::VideoWriter writer;
  const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
  if (!save_video) {
    //cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindowName, 1280, 720);
    cv::setWindowProperty(kWindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_FPS, 60);
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
#endif
  }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));

  //mediapipe::Packet useFaceDetectionSidePacket = mediapipe::MakePacket<bool>(false);

  //std::map<std::string, mediapipe::Packet> extra_side_packets = {
  //  {kUseFaceDetectionSidePacket, useFaceDetectionSidePacket}
  //};
  //MP_RETURN_IF_ERROR(graph.StartRun(extra_side_packets));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  //LOG(INFO) << "TEST ! Add side packet ! TEST.";
  //mediapipe::InputStreamHandler
  //MP_RETURN_IF_ERROR( graph.AddPacketToInputStream(kUseFaceDetectionSidePacket, useFaceDetectionSidePacket) );

  int age = 20;
  int cigarettes = 0;

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;
  while (grab_frames) {
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      LOG(INFO) << "Ignore empty frames from camera.";
      continue;
    }
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGBA);
    if (!load_video) {
      cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    }

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Prepare and add graph input packet.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(
        gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                   &gpu_helper]() -> absl::Status {
          // Convert ImageFrame to GpuBuffer.
          auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
          auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
          glFlush();
          texture.Release();
          // Send GPU image packet into the graph.
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(gpu_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp_us))));
          return absl::OkStatus();
        }));

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet)) break;
    std::unique_ptr<mediapipe::ImageFrame> output_frame;

    // Convert GpuBuffer to ImageFrame.
    MP_RETURN_IF_ERROR(gpu_helper.RunInGlContext(
        [&packet, &output_frame, &gpu_helper]() -> absl::Status {
          auto& gpu_frame = packet.Get<mediapipe::GpuBuffer>();
          auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
          output_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
              gpu_frame.width(), gpu_frame.height(),
              mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
          gpu_helper.BindFramebuffer(texture);
          const auto info = mediapipe::GlTextureInfoForGpuBufferFormat(
              gpu_frame.format(), 0, gpu_helper.GetGlVersion());
          glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
                       info.gl_type, output_frame->MutablePixelData());
          glFlush();
          texture.Release();
          return absl::OkStatus();
        }));

    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
    if (output_frame_mat.channels() == 4) {
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGBA2BGR);
    } else {
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    }

    cv::imshow(kWindowName, output_frame_mat);

    // get keyboard input
    const int pressed_key = cv::waitKey(5);

    if (arduino == NULL) {
      // if arduino connection could not be made, use keyboard input to change age and cigarettes
      // Press Q and W to increase/decrease cigarettes
      if (pressed_key == kKeyW && cigarettes < kMaxCigarettes) {
        cigarettes++;
      } else if (pressed_key == kKeyQ && cigarettes > kMinCigarettes) {
        cigarettes--;
      }

      // Press A and S to increase/decrease age
      if (pressed_key == kKeyA && age > kMinAge) {
        age--;
      } else if (pressed_key == kKeyS && age < kMaxAge) {
        age++;
      }

    } else {
      // read slider values from arduino
      SliderValues* sliderVals = getSliderValues(arduino);
      if (sliderVals != NULL) {
        age = sliderVals->val1;
        cigarettes = sliderVals->val2;
      }
    }

    LOG(INFO) << "AGE: " << age << ", CIGS: " << cigarettes;

    // Add selected filter index as input to graph
    int selected_filter = getFilterIndex(age, cigarettes);
    graph.AddPacketToInputStream( 
      kSelectedFilterInputStream, 
      mediapipe::MakePacket<int>(selected_filter)
                  .At(mediapipe::Timestamp(frame_timestamp_us))
    );

    // Press ESC or X to exit.
    if (pressed_key == kKeyESC || pressed_key == kKeyX) {
      grab_frames = false;
    }
  }

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kSelectedFilterInputStream)); // causes crash on exit, but alternative is to hang forever...
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
