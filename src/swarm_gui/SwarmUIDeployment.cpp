//
// Created by mel on 6/24/21.
//

#include "SwarmUIDeployment.h"
#include "SoftwareDistributor.h"
#include <wx/button.h>
#include <wx/choice.h>


using namespace std;


SwarmUIDeployment::SwarmUIDeployment(wxWindow * parent,
                               wxWindowID id = wxID_ANY,
                               const wxPoint & pos = wxDefaultPosition,
                               const wxSize & size = wxDefaultSize,
                               long style = wxTAB_TRAVERSAL,
                               const wxString & name = wxPanelNameStr) :
            wxPanel(parent, id, pos, size, style, name) {

    this->loadImages();

    this->SetBackgroundColour(wxColour(40, 190, 170));

    this->imageChoice = new wxChoice(
            this,
            wxID_ANY,
            wxPoint(40,48),
            wxSize(150, 40),
            this->nImages,
            this->imagesAvailable,
            0, wxDefaultValidator, _T("ID_DEPLOY_BUTTON"));

    this->installButton = new wxButton(
            this,
            wxID_ANY,
            _("Install"),
            wxPoint(240,48),
            wxSize(200, 40),
            0, wxDefaultValidator, _T("ID_CHOOSE_IMAGE"));

    // Annoyingly we have to use lambdas to capture `this` in order to make this object oriented
    //   It's probably an anti-pattern but I'm so used to Java that it feels better than using a static function
    imageChoice->Bind(wxEVT_COMMAND_CHOICE_SELECTED, [&](auto e) { this->onChoiceMade(e); });
    installButton->Bind(wxEVT_COMMAND_CHOICE_SELECTED, [&](auto e) { this->onInstallPressed(e); });
}

SwarmUIDeployment::~SwarmUIDeployment(){
    delete[] this->imagesAvailable;
    delete this->imageChoice;
    delete this->installButton;
}

void SwarmUIDeployment::loadImages() {
    vector<string> images = getImages();
    this->nImages = (int) images.size();
    this->imagesAvailable = new wxString[this->nImages]();
    for(int i = 0; i < this->nImages; i++) this->imagesAvailable[i] = wxString::FromAscii(images[i].c_str());
}

string SwarmUIDeployment::getSelectedImage() const {
    return this->imagesAvailable[this->imageChoice->GetSelection()].ToStdString();
}

void SwarmUIDeployment::onChoiceMade(wxCommandEvent & event) const {
    long size = getImageSize(getSelectedImage());
    installButton->SetLabel("Install (" + to_string(size) + " bytes)");
}

void SwarmUIDeployment::onInstallPressed(wxCommandEvent & event) const {


    // TODO: will need to link with ROS here
}
