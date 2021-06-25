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
//*)

IMPLEMENT_APP(SwarmUIApp);

bool SwarmUIApp::OnInit()
{
    wxInitAllImageHandlers();

    auto* Frame = new SwarmUIFrame(nullptr);
    Frame->Show();
    SetTopWindow(Frame);
    return true;
}
