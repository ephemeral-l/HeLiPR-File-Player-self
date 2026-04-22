#include "mainwindow.h"
#include <QApplication>
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <ros/ros.h>
#include <QProcess>
#include <sys/stat.h>
#include <cstdlib>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "file_player");
  ros::NodeHandle nh;

  if (argc >= 3 && std::string(argv[1]) == "--headless") {
    QCoreApplication a(argc, argv);
    QMutex m;
    ROSThread t(nullptr, &m);
    t.ros_initialize(nh);
    t.data_folder_path_ = argv[2];
    t.save_flag_ = true;
    t.play_flag_ = true;
    t.auto_start_flag_ = true;
    t.play_rate_ = 1000.0;
    t.start();
    t.Ready();
    while (ros::ok() && t.play_flag_) {
      usleep(100000);
    }
    // Drain: wait for bag file size to stabilize (workers done writing)
    std::string bag_path = t.data_folder_path_ + "/0.bag";
    uint64_t prev_size = 0;
    int stable_ticks = 0;
    while (ros::ok() && stable_ticks < 15) {
      usleep(200000);
      struct stat st;
      if (stat(bag_path.c_str(), &st) != 0) {
        stable_ticks = 0;
        continue;
      }
      if ((uint64_t)st.st_size == prev_size) {
        stable_ticks++;
      } else {
        stable_ticks = 0;
      }
      prev_size = st.st_size;
    }
    if (t.process_flag_) {
      t.bag_.close();
      t.process_flag_ = false;
    }
    // DataStampThread's post-end busy-wait doesn't check active_, so
    // the destructor's join() would hang. Skip destructors.
    exit(0);
  }

  QApplication a(argc, argv);
  MainWindow w;
  w.RosInit(nh);
  w.show();

   return a.exec();
}
