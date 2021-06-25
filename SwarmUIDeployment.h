//
// Created by miles on 6/24/21.
//

#include <wx/panel.h>
#include <wx/button.h>
#include <wx/choice.h>

#ifndef SWARM_PROJECT_SWARMUIDEPLOYMENT_H
#define SWARM_PROJECT_SWARMUIDEPLOYMENT_H

class SwarmUIDeployment: public wxPanel {
public:

    wxString* imagesAvailable = {};
    int nImages = 0;

    wxChoice* imageChoice;
    wxButton* installButton;

    SwarmUIDeployment(wxWindow *parent, wxWindowID id, const wxPoint &pos, const wxSize &size, long style,
                      const wxString &name);
    ~SwarmUIDeployment() override;

    void onChoiceMade(wxCommandEvent &event) const;

    void loadImages();
};


#endif //SWARM_PROJECT_SWARMUIDEPLOYMENT_H
