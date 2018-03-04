#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void findInterceptLocation(double hunter_x, double hunter_y, UKF* ukf, double& intercept_x, double& intercept_y){
  VectorXd x = ukf -> x_;
  MatrixXd P = ukf -> P_;

  float t_max = 8;
  float t_min = 0;
  float t_mid = 0;
  double target_x = x[0];
  double target_y = x[1];
  double dist_target_intercept;
  double time_intercept;
  double intercept_try_x;
  double intercept_try_y;
  double best_deal = 1000;
  double max_v = 5;

  int max_iter = 10;
  bool find_intercept_loc = false;

  while(t_max > t_min && max_iter > 0){
    //restore ukf x and P parameter
    ukf -> x_ = x;
    ukf -> P_ = P;

    t_mid = (t_max + t_min)/2;
    ukf->Prediction(t_mid);

    intercept_try_x = ukf->x_[0];
    intercept_try_y = ukf->x_[1];

    dist_target_intercept = sqrt((hunter_x - intercept_try_x) * (hunter_x - intercept_try_x)
                                 + (hunter_y - intercept_try_y) * (hunter_y - intercept_try_y));
    time_intercept = dist_target_intercept / max_v;

    double diff_time_intercept = fabs(t_mid - time_intercept);

    if(diff_time_intercept < best_deal){
      best_deal= diff_time_intercept;
      intercept_x = intercept_try_x;
      intercept_y = intercept_try_y;
    }

    /*
    if(diff_time_intercept < 0.002){
      find_intercept_loc = true;
      std::cout << "Intercept: Find interception at time: " << t_mid << " intercept at: " << intercept_try_x << " " << intercept_try_y << std::endl;

      intercept_x = intercept_try_x;
      intercept_y = intercept_try_y;
      break;
    }*/

    if(time_intercept > t_mid)
      t_min = t_mid;
    else if (time_intercept < t_mid)
      t_max = t_mid;

    --max_iter;
  }

  ukf -> x_ = x;
  ukf -> P_ = P;
}

void findInterceptLocation2(long timestamp, double hunter_x, double hunter_y, UKF* ukf, double& intercept_x, double& intercept_y){
  VectorXd x = ukf -> x_;
  MatrixXd P = ukf -> P_;

  float t_max = 8;
  float t_min = 0;
  float t_mid = 0;
  double target_x = x[0];
  double target_y = x[1];
  double dist_target_intercept;
  double time_intercept;
  double intercept_try_x;
  double intercept_try_y;
  double best_deal = 1000;
  double max_v = 5;

  bool find_intercept_loc = false;

  for(int i=0; i< 8*5; ++i){
    t_mid = i*1.0/5;
    //restore ukf x and P parameter
    ukf -> x_ = x;
    ukf -> P_ = P;

    ukf->Prediction(t_mid);

    intercept_try_x = ukf->x_[0];
    intercept_try_y = ukf->x_[1];

    dist_target_intercept = sqrt((hunter_x - intercept_try_x) * (hunter_x - intercept_try_x)
                                 + (hunter_y - intercept_try_y) * (hunter_y - intercept_try_y));
    time_intercept = dist_target_intercept / max_v;

    double diff_time_intercept = fabs(t_mid - time_intercept);

    //std::cout << timestamp << " " << t_mid << " " << time_intercept << " " << diff_time_intercept << endl;

    if(best_deal > diff_time_intercept){
      intercept_x = intercept_try_x;
      intercept_y = intercept_try_y;
    }

  }

  ukf -> x_ = x;
  ukf -> P_ = P;

}

int main()
{
  uWS::Hub h;

  // Create a UKF instance
  UKF ukf;

  double target_x = 0.0;
  double target_y = 0.0;

  h.onMessage([&ukf,&target_x,&target_y](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {


        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());

          string lidar_measurment = j[1]["lidar_measurement"];

          MeasurementPackage meas_package_L;
          istringstream iss_L(lidar_measurment);
          long long timestamp_L;

          // reads first element from the current line
          string sensor_type_L;
          iss_L >> sensor_type_L;

      	  // read measurements at this timestamp
      	  meas_package_L.sensor_type_ = MeasurementPackage::LASER;
          meas_package_L.raw_measurements_ = VectorXd(2);
          float px;
      	  float py;
          iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;

    	    ukf.ProcessMeasurement(meas_package_L);

          //std::cout << "L" << px << " " << py << std::endl;
          //std::cout << "L" << ukf.x_[0] << " " << ukf.x_[1] << std::endl;

    	    string radar_measurment = j[1]["radar_measurement"];

          MeasurementPackage meas_package_R;
          istringstream iss_R(radar_measurment);
    	    long long timestamp_R;

          // reads first element from the current line
          string sensor_type_R;
          iss_R >> sensor_type_R;

      	  // read measurements at this timestamp
      	  meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;

          ukf.ProcessMeasurement(meas_package_R);

          target_x = ukf.x_[0];
          target_y = ukf.x_[1];

          //std::cout << "R" << ro << " " << theta << " " << ro_dot << std::endl;
          //std::cout << "R" << ukf.x_[0] << " " << ukf.x_[1] << std::endl;

          //find interception point
          double intercept_x;
          double intercept_y;
          findInterceptLocation(hunter_x, hunter_y, &ukf, intercept_x, intercept_y);
          //intercept_x = target_x;
          //intercept_y = target_y;

          double heading_to_target = atan2(intercept_y - hunter_y, intercept_x - hunter_x);
          while (heading_to_target > M_PI) heading_to_target-=2.*M_PI;
          while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
          //turn towards the target
          double heading_difference = heading_to_target - hunter_heading;
          while (heading_difference > M_PI) heading_difference-=2.*M_PI;
          while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

          double distance_difference = sqrt((intercept_y - hunter_y)*(intercept_y - hunter_y) + (intercept_x - hunter_x)*(intercept_x - hunter_x));
          distance_difference = fmin(fmax(distance_difference, -1000), 1000);

          json msgJson;
          msgJson["turn"] = heading_difference;
          msgJson["dist"] = distance_difference;
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































