//
// Created by miles on 6/24/21.
//

#include <wx/panel.h>
#include <wx/button.h>
#include <wx/choice.h>
#include <ros/ros.h>

#ifndef SWARM_PROJECT_SWARMUIDEPLOYMENT_H
#define SWARM_PROJECT_SWARMUIDEPLOYMENT_H

class SwarmUIDeployment: public wxPanel {
public:

    wxString* imagesAvailable = {};
    wxChoice* imageChoice;
    wxButton* installButton;

    int nImages = 0;

    int serverSocket;

    SwarmUIDeployment(wxWindow *parent, wxWindowID id, const wxPoint &pos, const wxSize &size, long style,
                      const wxString &name);
    ~SwarmUIDeployment() override;

    void onChoiceMade(wxCommandEvent &event) const;

    void loadImages();

    void onInstallPressed(wxCommandEvent &event) const;

    std::string getSelectedImage() const;

};


#endif //SWARM_PROJECT_SWARMUIDEPLOYMENT_H
