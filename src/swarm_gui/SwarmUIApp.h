/***************************************************************
 * Name:      SwarmUIApp.h
 * Purpose:   Defines Application Class
 * Author:    Chandima Fernando (wfernando@huskers.unl.edu)
 * Created:   2021-06-19
 * Copyright: Chandima Fernando ()
 * License:
 **************************************************************/

#ifndef SWARMUIAPP_H
#define SWARMUIAPP_H

#include <wx/app.h>

class SwarmUIApp : public wxApp
{
    public:
        bool OnInit() override;
};

#endif // SWARMUIAPP_H
