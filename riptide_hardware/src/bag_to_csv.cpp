#include "riptide_hardware/bag_to_csv.h"


//!!!!!!!!!!!!!!!!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!///
// Must extract /resources/FrontPanel. Run install.sh as sudo //
// and sudo copy API/okFrontPanel.so file to /usr/lib		  //
////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_to_csv");
  Acoustics ac;
  ros::spin();
}

Acoustics::Acoustics() : nh("bag_to_csv") { // NOTE: there is no namespace declared in nh()
	acoustics_sub = nh.subscribe<riptide_msgs::Acoustics>("/state/acoustics", 1, &Acoustics::Callback, this);
    LoadParam<string>("username", username);
    LoadParam<string>("file_name", file_name);
    LoadParam<string>("ext", ext);
}

template <typename T>
void Acoustics::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Extract Video Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void Acoustics::Callback(const riptide_msgs::Acoustics::ConstPtr& ac)
{
    std::ofstream outfile;
    outfile.open("/home/"+username+"/"+file_name+"."+ext, std::ios_base::app);
    outfile << ac->PFphase << "," << ac->PAphase << "," << ac->SFphase << "," << ac->SAphase << "\n"; 
    outfile.close();
}
