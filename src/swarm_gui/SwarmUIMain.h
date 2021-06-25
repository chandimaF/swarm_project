/***************************************************************
 * Name:      SwarmUIMain.h
 * Purpose:   Defines Application Frame
 * Author:    Chandima Fernando (wfernando@huskers.unl.edu)
 * Created:   2021-06-19
 * Copyright: Chandima Fernando ()
 * License:
 **************************************************************/

#ifndef SWARMUIMAIN_H
#define SWARMUIMAIN_H

//(*Headers(SwarmUIFrame)
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
//*)

class SwarmUIFrame: public wxFrame
{
    public:

        SwarmUIFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~SwarmUIFrame();

    private:

        //(*Handlers(SwarmUIFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnButton1Click(wxCommandEvent& event);
        void OnNotebook1PageChanged(wxNotebookEvent& event);
        void OnButton2Click(wxCommandEvent& event);
        //*)

        //(*Identifiers(SwarmUIFrame)
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_PANEL1;
        static const long ID_PANEL2;
        static const long ID_PANEL3;
        static const long ID_PANEL4;
        static const long ID_PANEL5;
        static const long ID_NOTEBOOK1;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(SwarmUIFrame)
        wxButton* Button1;
        wxButton* Button2;
        wxNotebook* Notebook1;
        wxPanel* Panel1;
        wxPanel* Panel2;
        wxPanel* Panel3;
        wxPanel* Panel4;
        wxPanel* Panel5;
        wxStatusBar* StatusBar1;
        //*)

        DECLARE_EVENT_TABLE()
};

#endif // SWARMUIMAIN_H
