/***************************************************************
 * Name:      SwarmUIMain.cpp
 * Purpose:   Code for Application Frame
 * Author:    Chandima Fernando (wfernando@huskers.unl.edu)
 * Created:   2021-06-19
 * Copyright: Chandima Fernando ()
 * License:
 **************************************************************/

#include "SwarmUIMain.h"
#include "SwarmUIDeployment.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(SwarmUIFrame)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(SwarmUIFrame)
const long SwarmUIFrame::ID_BUTTON1 = wxNewId();
const long SwarmUIFrame::ID_BUTTON2 = wxNewId();
const long SwarmUIFrame::ID_PANEL1 = wxNewId();
const long SwarmUIFrame::ID_PANEL2 = wxNewId();
const long SwarmUIFrame::ID_PANEL3 = wxNewId();
const long SwarmUIFrame::ID_PANEL4 = wxNewId();
const long SwarmUIFrame::ID_PANEL5 = wxNewId();
const long SwarmUIFrame::ID_NOTEBOOK1 = wxNewId();
const long SwarmUIFrame::idMenuQuit = wxNewId();
const long SwarmUIFrame::idMenuAbout = wxNewId();
const long SwarmUIFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(SwarmUIFrame,wxFrame)
    //(*EventTable(SwarmUIFrame)
    //*)
END_EVENT_TABLE()

SwarmUIFrame::SwarmUIFrame(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(SwarmUIFrame)
    wxMenu* Menu1;
    wxMenu* Menu2;
    wxMenuBar* MenuBar1;
    wxMenuItem* MenuItem1;
    wxMenuItem* MenuItem2;

    Create(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxPoint(224,24), wxSize(520,552), wxNB_LEFT, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxPoint(-13,-4), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    Button1 = new wxButton(Panel1, ID_BUTTON1, _("Click Me"), wxPoint(40,48), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    Button2 = new wxButton(Panel1, ID_BUTTON2, _("Don\'t Click Me"), wxPoint(168,48), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    Panel4 = new wxPanel(Notebook1, ID_PANEL2, wxPoint(249,13), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxPoint(13,173), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    Panel2 = new SwarmUIDeployment(Notebook1, ID_PANEL4, wxPoint(34,181), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    Panel5 = new wxPanel(Notebook1, ID_PANEL5, wxPoint(37,158), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    Notebook1->AddPage(Panel1, _("Test Tab"), false);
    Notebook1->AddPage(Panel2, _("Deployment"), false);
    Notebook1->AddPage(Panel3, _("Simulation "), false);
    Notebook1->AddPage(Panel4, _("Parameters"), false);
    Notebook1->AddPage(Panel5, _("Control Station"), false);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&SwarmUIFrame::OnButton1Click);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&SwarmUIFrame::OnButton2Click);
    Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED,(wxObjectEventFunction)&SwarmUIFrame::OnNotebook1PageChanged);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&SwarmUIFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&SwarmUIFrame::OnAbout);
    //*)
}

SwarmUIFrame::~SwarmUIFrame()
{
    //(*Destroy(SwarmUIFrame)
    //*)
}

void SwarmUIFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void SwarmUIFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to..."));
}

void SwarmUIFrame::OnButton1Click(wxCommandEvent& event)
{
    wxMessageBox(_("Hello NIMBUS"));
}

void SwarmUIFrame::OnNotebook1PageChanged(wxNotebookEvent& event)
{
    // Do nothing
}

void SwarmUIFrame::OnButton2Click(wxCommandEvent& event)
{
    wxMessageBox("But you did :-/ ");
}
