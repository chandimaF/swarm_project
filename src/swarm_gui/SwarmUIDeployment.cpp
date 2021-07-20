//
// Created by mel on 6/24/21.
//

#include "SwarmUIDeployment.h"
#include "SoftwareDistributor.h"
#include <wx/button.h>
#include <wx/choice.h>
#include <wx/checkbox.h>
#include <wx/stattext.h>


using namespace std;


SwarmUIDeployment::SwarmUIDeployment(wxWindow * parent,
                               wxWindowID id = wxID_ANY,
                               const wxPoint & pos = wxDefaultPosition,
                               const wxSize & size = wxDefaultSize,
                               long style = wxTAB_TRAVERSAL,
                               const wxString & name = wxPanelNameStr) :
            wxPanel(parent, id, pos, size, style, name) {

    this->loadImages();
    this->loadAgents();

    this->SetBackgroundColour(wxColour(40, 190, 170));

    new wxStaticText (this, wxID_ANY, _("Project: "), wxPoint(60, 60), wxSize(100, 40));
    this->imageChoice = new wxChoice(
            this,
            wxID_ANY,
            wxPoint(120,50),
            wxSize(200, 40),
            this->nImages,
            this->imagesAvailable);

    new wxStaticText (this, wxID_ANY, _("Agent: "), wxPoint(60, 110), wxSize(100, 40));
    this->agentChoice = new wxChoice(
            this,
            wxID_ANY,
            wxPoint(120,100),
            wxSize(200, 40),
            3,
            this->agentsAvailable);

    this->installButton = new wxButton(
            this,
            wxID_ANY,
            _("Install"),
            wxPoint(60,150),
            wxSize(200, 40));

    this->checkFullPatch = new wxCheckBox(
            this,
            wxID_ANY,
            _(""),
            wxPoint(280, 160),
            wxSize(20, 20));

    // Annoyingly we have to use lambdas to capture `this` in order to make this object oriented
    //   It's probably an anti-pattern but I'm so used to Java that it feels better than using a static function
    imageChoice->Bind(wxEVT_COMMAND_CHOICE_SELECTED, [&](auto e) { this->onChoiceMade(e); });
    installButton->Bind(wxEVT_COMMAND_CHOICE_SELECTED, [&](auto e) { this->onInstallPressed(e); });
}

SwarmUIDeployment::~SwarmUIDeployment(){
    delete[] this->imagesAvailable;
    delete this->imageChoice;
    delete this->installButton;
    delete this->checkFullPatch;
}

void SwarmUIDeployment::loadImages() {
    vector<string> images = getImages();
    this->nImages = (int) images.size();
    this->imagesAvailable = new wxString[this->nImages]();
    for(int i = 0; i < this->nImages; i++) this->imagesAvailable[i] = wxString::FromAscii(images[i].c_str());
}

void SwarmUIDeployment::loadAgents() {
    this->agentsAvailable = new wxString[3]();
    this->agentsAvailable[0] = _("agent_1");
    this->agentsAvailable[1] = _("agent_2");
    this->agentsAvailable[2] = _("agent_3");
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
