/***************************************************************
 * Name:      SwarmUIApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Chandima Fernando (wfernando@huskers.unl.edu)
 * Created:   2021-06-19
 * Copyright: Chandima Fernando ()
 * License:
 **************************************************************/

#include "SwarmUIApp.h"

//(*AppHeaders
#include "SwarmUIMain.h"
#include <wx/image.h>
#include <ros/ros.h>
//*)

IMPLEMENT_APP(SwarmUIApp);

bool SwarmUIApp::OnInit()
{
    int argc = 0; char * argv[] = {};
    ros::init(argc, argv, "swarm_gui");

    wxInitAllImageHandlers();

    auto* Frame = new SwarmUIFrame(nullptr);
    Frame->Show();
    SetTopWindow(Frame);
    return true;
}
