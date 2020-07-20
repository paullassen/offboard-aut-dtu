#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>    // std::stringstream
#include <stdexcept>  // std::runtime_error
#include <string>
#include <utility>  // std::pair
#include <vector>

std::vector<std::pair<std::string, std::vector<long double> > > read_csv(
    std::string filename) {
  // Reads a CSV file into a vector of <string, vector<int>> pairs where
  // each pair represents <column name, column values>

  // Create a vector of <string, int vector> pairs to store the result
  std::vector<std::pair<std::string, std::vector<long double> > > result;

  // Create an input filestream
  std::ifstream myFile(filename, std::ifstream::in);

  // Make sure the file is open
  if (!myFile.is_open()) throw std::runtime_error("Could not open file");

  // Helper vars
  std::string line, colname;
  long double val;

  // Read the column names
  if (myFile.good()) {
    // Extract the first line in the file
    std::getline(myFile, line);

    // Create a stringstream from line
    std::stringstream ss(line);

    // Extract each column name
    while (std::getline(ss, colname, ',')) {
      std::cout << colname << std::endl;
      // Initialize and add <colname, int vector> pairs to result
      result.push_back({colname, std::vector<long double>{}});
    }
  }

  // Read data, line by line
  while (std::getline(myFile, line)) {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Keep track of the current column index
    int colIdx = 0;

    // Extract each integer
    while (std::getline(ss, colname, ',')) {
      if (std::stringstream(colname) >> val) {
        result.at(colIdx).second.push_back(val);
      } else {
        result.at(colIdx).second.push_back(-110);
      }
      // Increment the column index
      colIdx++;
    }
  }

  // Close file
  myFile.close();

  return result;
}

void write_csv(
    std::string filename,
    std::vector<std::pair<std::string, std::vector<long double> > > dataset) {
  // Make a CSV file with one or more columns of integer values
  // Each column of data is represented by the pair <column name, column data>
  //   as std::pair<std::string, std::vector<int>>
  // The dataset is represented as a vector of these columns
  // Note that all columns should be the same size

  // Create an output filestream object
  std::ofstream myFile(filename, std::ofstream::out);
  // Send column names to the stream

  myFile << "roll, pitch, yaw\n";
  for (int i = 0; i < dataset.at(0).second.size(); ++i) {
    int j = dataset.size() - 4;
    int k = j + 1;
    int l = k + 1;
    int m = l + 1;
    tf::Quaternion q(dataset.at(j).second.at(i), dataset.at(k).second.at(i),
                     dataset.at(l).second.at(i), dataset.at(m).second.at(i));
    tf::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    myFile << std::setw(25) << std::setprecision(25) << roll;
    myFile << ",";
    myFile << std::setw(25) << std::setprecision(25) << pitch;
    myFile << ",";
    myFile << std::setw(25) << std::setprecision(25) << yaw;
    myFile << "\n";
  }

  // Close the file
  myFile.close();
}

int main(int argc, char** argv) {
  for (int i = 0; i < argc; ++i) {
    std::cout << i << "\t" << argc << "\t" << argv[i] << std::endl;
  }
  std::string path_ = "/home/pala/Documents/MATLAB/bagger/";
  std::string input = "/bagfile-_test_mocap.csv";
  std::string output = "/rpy.csv";
  std::stringstream in, out;
  in << path_ << std::string((argc > 1) ? argv[1] : "test17") << input;
  out << path_ << std::string((argc > 1) ? argv[1] : "test17") << output;

  std::string path =
      "/home/pala/Documents/MATLAB/bagger/test17/bagfile-_test_mocap.csv";
  // Read three_cols.csv and ones.csv
  std::vector<std::pair<std::string, std::vector<long double> > > rpy =
      read_csv(in.str());
  // std::vector<std::pair<std::string, std::vector<int> > > ones =
  // read_csv("ones.csv");
  std::cout << "Done Reading" << std::endl;
  // Write to another file to check that this was successful
  write_csv(out.str(), rpy);
  // write_csv("ones_copy.csv", ones);

  return 0;
}

