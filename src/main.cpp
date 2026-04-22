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
#include <dirent.h>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <algorithm>

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
    t.pause_flag_ = false;  // ROSThread ctor leaves this uninitialized; TimerCallback gates on it
    t.auto_start_flag_ = true;
    t.play_rate_ = 1000.0;
    t.start();

    // Estimate final bag size from sum of LiDAR .bin file sizes (for progress bar)
    std::cout << "Scanning LiDAR folders..." << std::flush;
    uint64_t estimate_total = 0;
    uint64_t file_count = 0;
    for (const std::string& name : {"/LiDAR/Ouster", "/LiDAR/Velodyne", "/LiDAR/Avia", "/LiDAR/Aeva"}) {
      std::string path = t.data_folder_path_ + name;
      DIR* d = opendir(path.c_str());
      if (d == nullptr) {
        continue;
      }
      while (dirent* e = readdir(d)) {
        if (e->d_name[0] == '.') {
          continue;
        }
        struct stat sst;
        if (stat((path + "/" + e->d_name).c_str(), &sst) == 0) {
          estimate_total += sst.st_size;
          file_count++;
        }
      }
      closedir(d);
    }
    std::cout << " " << file_count << " files, "
              << std::fixed << std::setprecision(2)
              << (estimate_total / 1e9) << " GB" << std::endl;

    t.Ready();

    std::string bag_path = t.data_folder_path_ + "/0.bag";
    auto tstart = std::chrono::steady_clock::now();
    uint64_t prev_size = 0;
    int stable_ticks = 0;
    bool scheduler_done = false;
    while (ros::ok() && stable_ticks < 15) {
      usleep(200000);
      struct stat st;
      uint64_t bag_size = (stat(bag_path.c_str(), &st) == 0) ? (uint64_t)st.st_size : 0;
      if (!t.play_flag_) {
        scheduler_done = true;
      }
      if (scheduler_done) {
        stable_ticks = (bag_size == prev_size) ? stable_ticks + 1 : 0;
      }
      prev_size = bag_size;

      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - tstart).count();
      double mbps = (elapsed > 0) ? (bag_size / 1e6 / elapsed) : 0.0;
      double frac = (estimate_total > 0) ? std::min(1.0, (double)bag_size / (double)estimate_total) : 0.0;
      double eta = (mbps > 1.0 && frac < 0.99) ? ((double)(estimate_total - bag_size) / 1e6 / mbps) : 0.0;

      const int bar_w = 30;
      int filled = (int)(frac * bar_w);
      std::cout << "\r["
                << std::string(filled, '=') << std::string(bar_w - filled, ' ')
                << "] "
                << std::fixed << std::setprecision(1) << (frac * 100.0) << "%  "
                << std::setprecision(2) << (bag_size / 1e9) << "/"
                << (estimate_total / 1e9) << " GB  "
                << std::setprecision(1) << mbps << " MB/s  ";
      if (scheduler_done) {
        std::cout << "draining " << stable_ticks << "/15      ";
      } else {
        std::cout << "ETA " << (int)eta << "s         ";
      }
      std::cout << std::flush;
    }
    std::cout << std::endl;

    if (t.process_flag_) {
      t.bag_.close();
      t.process_flag_ = false;
    }
    std::cout << "Bag written: " << bag_path << " ("
              << std::fixed << std::setprecision(2) << (prev_size / 1e9) << " GB)" << std::endl;
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
