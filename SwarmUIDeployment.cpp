//
// Created by mel on 6/24/21.
//

#include "SwarmUIDeployment.h"
#include <wx/button.h>
#include <wx/choice.h>
#include "SoftwareDistributor.h"

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

    this->imageChoice = new wxChoice(this, wxID_ANY, wxPoint(40,48), wxSize(150, 40), this->nImages, this->imagesAvailable, 0, wxDefaultValidator, _T("ID_DEPLOY_BUTTON"));
    this->installButton = new wxButton(this, wxID_ANY, _("Install"), wxPoint(240,48), wxSize(200, 40), 0, wxDefaultValidator, _T("ID_CHOOSE_IMAGE"));
    imageChoice->Bind(wxEVT_COMMAND_CHOICE_SELECTED, [&](auto e) { this->onChoiceMade(e); });
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

void SwarmUIDeployment::onChoiceMade(wxCommandEvent & event) const {
    long size = getImageSize(this->imagesAvailable[this->imageChoice->GetSelection()].ToStdString());
    installButton->SetLabel("Install (" + to_string(size) + " bytes)");
}
