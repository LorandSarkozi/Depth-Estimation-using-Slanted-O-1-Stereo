#include "stdafx.h"
#include <wx/wx.h>
#include <wx/filedlg.h>
#include <opencv2/opencv.hpp>
#include "sos.h"
#include<fstream>


using namespace cv;
using namespace std;

class DepthApp : public wxApp {
public:
    virtual bool OnInit();
};

class DepthFrame : public wxFrame {
public:
    DepthFrame(const wxString& title);

private:
    wxTextCtrl* leftPathCtrl;
    wxTextCtrl* rightPathCtrl;
    wxTextCtrl* gtPathCtrl;
    wxTextCtrl* maxDispCtrl;
    wxStaticText* scoreText;
    wxStaticBitmap* disparityBitmap;
    wxStaticBitmap* groundTruthBitmap;
    wxStaticBitmap* errorBitmap;
    Mat disparityMap;
    wxPanel* panel;

    void OnBrowseLeft(wxCommandEvent&);
    void OnBrowseRight(wxCommandEvent&);
    void OnBrowseGT(wxCommandEvent&);
    void OnRun(wxCommandEvent&);
    void OnSaveDisparity(wxCommandEvent&);
    void OnReset(wxCommandEvent&);

    wxDECLARE_EVENT_TABLE();
};

enum {
    ID_BROWSE_LEFT = 1,
    ID_BROWSE_RIGHT,
    ID_BROWSE_GT,
    ID_RUN,
    ID_SAVE_DISPARITY,
    ID_RESET_FIELDS
};

wxBEGIN_EVENT_TABLE(DepthFrame, wxFrame)
EVT_BUTTON(ID_BROWSE_LEFT, DepthFrame::OnBrowseLeft)
EVT_BUTTON(ID_BROWSE_RIGHT, DepthFrame::OnBrowseRight)
EVT_BUTTON(ID_BROWSE_GT, DepthFrame::OnBrowseGT)
EVT_BUTTON(ID_RUN, DepthFrame::OnRun)
EVT_BUTTON(ID_SAVE_DISPARITY, DepthFrame::OnSaveDisparity)
EVT_BUTTON(ID_RESET_FIELDS, DepthFrame::OnReset)
wxEND_EVENT_TABLE()

wxIMPLEMENT_APP(DepthApp);

bool DepthApp::OnInit() {
    DepthFrame* frame = new DepthFrame("Slanted O(1) Stereo");
    frame->Show(true);
    return true;
}

DepthFrame::DepthFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 1000)) {

    void OnSaveDisparity(wxCommandEvent&);
    void OnReset(wxCommandEvent&);

    panel = new wxPanel(this);
    wxBoxSizer* vbox = new wxBoxSizer(wxVERTICAL);

    wxBoxSizer* fileSizer1 = new wxBoxSizer(wxHORIZONTAL);
    leftPathCtrl = new wxTextCtrl(panel, wxID_ANY);
    fileSizer1->Add(new wxStaticText(panel, wxID_ANY, "Left Image:"), 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    fileSizer1->Add(leftPathCtrl, 1, wxALL | wxEXPAND, 5);
    fileSizer1->Add(new wxButton(panel, ID_BROWSE_LEFT, "Browse"), 0, wxALL, 5);

    wxBoxSizer* fileSizer2 = new wxBoxSizer(wxHORIZONTAL);
    rightPathCtrl = new wxTextCtrl(panel, wxID_ANY);
    fileSizer2->Add(new wxStaticText(panel, wxID_ANY, "Right Image:"), 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    fileSizer2->Add(rightPathCtrl, 1, wxALL | wxEXPAND, 5);
    fileSizer2->Add(new wxButton(panel, ID_BROWSE_RIGHT, "Browse"), 0, wxALL, 5);

    wxBoxSizer* fileSizer3 = new wxBoxSizer(wxHORIZONTAL);
    gtPathCtrl = new wxTextCtrl(panel, wxID_ANY);
    fileSizer3->Add(new wxStaticText(panel, wxID_ANY, "Ground Truth:"), 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    fileSizer3->Add(gtPathCtrl, 1, wxALL | wxEXPAND, 5);
    fileSizer3->Add(new wxButton(panel, ID_BROWSE_GT, "Browse"), 0, wxALL, 5);

    wxBoxSizer* controlSizer = new wxBoxSizer(wxHORIZONTAL);
    maxDispCtrl = new wxTextCtrl(panel, wxID_ANY, "256");
    controlSizer->Add(new wxStaticText(panel, wxID_ANY, "Max Disparity:"), 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlSizer->Add(maxDispCtrl, 0, wxALL, 5);
    controlSizer->Add(new wxButton(panel, ID_RUN, "Run SOS"), 0, wxALL, 5);
    controlSizer->Add(new wxButton(panel, ID_SAVE_DISPARITY, "Save"), 0, wxALL, 5);
    controlSizer->Add(new wxButton(panel, ID_RESET_FIELDS, "Reset"), 0, wxALL, 5);

    scoreText = new wxStaticText(panel, wxID_ANY, "Score: ");
    disparityBitmap = new wxStaticBitmap(panel, wxID_ANY, wxBitmap(1, 1)); // placeholder
    groundTruthBitmap = new wxStaticBitmap(panel, wxID_ANY, wxBitmap(1, 1));
    errorBitmap = new wxStaticBitmap(panel, wxID_ANY, wxBitmap(1, 1));

    wxFont boldFont(wxFontInfo(10).Bold());

    wxStaticText* disparityLabel = new wxStaticText(panel, wxID_ANY, "Disparity Map");
    disparityLabel->SetFont(boldFont);

    wxStaticText* groundTruthLabel = new wxStaticText(panel, wxID_ANY, "Ground Truth");
    groundTruthLabel->SetFont(boldFont);

    wxStaticText* errorLabel = new wxStaticText(panel, wxID_ANY, "Error");
    errorLabel->SetFont(boldFont);


    vbox->Add(fileSizer1, 0, wxEXPAND);
    vbox->Add(fileSizer2, 0, wxEXPAND);
    vbox->Add(fileSizer3, 0, wxEXPAND);
    vbox->Add(controlSizer, 0, wxEXPAND);
    vbox->Add(scoreText, 0, wxALL, 10);
    //vbox->Add(disparityBitmap, 1, wxEXPAND | wxALL, 10);
    //vbox->Add(groundTruthBitmap, 1, wxEXPAND | wxALL, 10);

    wxBoxSizer* disparityBox = new wxBoxSizer(wxVERTICAL);
    disparityBox->Add(disparityLabel, 0, wxALIGN_CENTER | wxBOTTOM, 10);  
    disparityBox->Add(disparityBitmap, 0, wxALIGN_CENTER | wxTOP | wxBOTTOM, 10); 

    wxBoxSizer* gtBox = new wxBoxSizer(wxVERTICAL);
    gtBox->Add(groundTruthLabel, 0, wxALIGN_CENTER | wxBOTTOM, 10);
    gtBox->Add(groundTruthBitmap, 0, wxALIGN_CENTER | wxTOP | wxBOTTOM, 10);

    wxBoxSizer* errorBox = new wxBoxSizer(wxVERTICAL);
    errorBox->Add(errorLabel, 0, wxALIGN_CENTER | wxBOTTOM, 10);
    errorBox->Add(errorBitmap, 0, wxALIGN_CENTER | wxTOP | wxBOTTOM, 10);

    wxBoxSizer* imagesRow = new wxBoxSizer(wxHORIZONTAL);
    imagesRow->Add(gtBox, 1, wxALL, 15);  
    imagesRow->AddSpacer(10);                   
    imagesRow->Add(disparityBox, 1, wxALL, 15);
    imagesRow->AddSpacer(10);
    imagesRow->Add(errorBox, 1, wxALL, 15);

    vbox->Add(imagesRow, 2, wxALIGN_CENTER | wxALL, 10);
    

    
    panel->SetSizer(vbox);
}

void DepthFrame::OnSaveDisparity(wxCommandEvent&) {
    if (disparityMap.empty()) {
        wxMessageBox("Nu este disponibila nicio harta de disparitate. Va rugam sa rulati mai intai algoritmul SOS.", "Error");
        return;
    }

    wxFileDialog saveFileDialog(this, "Save Disparity Image", "", "disparity.png", "PNG files (*.png)|*.png", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (saveFileDialog.ShowModal() == wxID_CANCEL) return;

    if (!imwrite(saveFileDialog.GetPath().ToStdString(), disparityMap)) {
        wxMessageBox("Failed to save image.", "Error");
    }
}

void DepthFrame::OnReset(wxCommandEvent&) {
    leftPathCtrl->Clear();
    rightPathCtrl->Clear();
    gtPathCtrl->Clear();
    maxDispCtrl->SetValue("256");

    scoreText->SetLabel("Score: ");

    wxBitmap blank(1, 1);
    disparityBitmap->SetBitmap(blank);
    groundTruthBitmap->SetBitmap(blank);
    errorBitmap->SetBitmap(blank);

    disparityBitmap->Refresh();
    groundTruthBitmap->Refresh();
    errorBitmap->Refresh();

    Layout();
    panel->Layout();
    panel->Refresh();
}



void DepthFrame::OnBrowseLeft(wxCommandEvent&) {
    wxFileDialog dlg(this, "Select Left Image", "", "", "Images (*.png;*.jpg)|*.png;*.jpg", wxFD_OPEN);
    if (dlg.ShowModal() == wxID_OK)
        leftPathCtrl->SetValue(dlg.GetPath());
}

void DepthFrame::OnBrowseRight(wxCommandEvent&) {
    wxFileDialog dlg(this, "Select Right Image", "", "", "Images (*.png;*.jpg)|*.png;*.jpg", wxFD_OPEN);
    if (dlg.ShowModal() == wxID_OK)
        rightPathCtrl->SetValue(dlg.GetPath());
}

void DepthFrame::OnBrowseGT(wxCommandEvent&) {
    wxFileDialog dlg(this, "Select Ground Truth", "", "", "Images (*.png;*.jpg)|*.png;*.jpg", wxFD_OPEN);
    if (dlg.ShowModal() == wxID_OK)
        gtPathCtrl->SetValue(dlg.GetPath());
}

void DepthFrame::OnRun(wxCommandEvent&) {
    string leftPath = leftPathCtrl->GetValue().ToStdString();
    string rightPath = rightPathCtrl->GetValue().ToStdString();
    string gtPath = gtPathCtrl->GetValue().ToStdString();
    int maxDisp = stoi(maxDispCtrl->GetValue().ToStdString());

    Mat left = imread(leftPath);
    Mat right = imread(rightPath);
    Mat gt = imread(gtPath, IMREAD_GRAYSCALE);
    if (left.empty() || right.empty() || gt.empty()) {
        wxMessageBox("Una sau mai multe imagini nu au putut fi incarcate.", "Error");
        return;
    }

    disparityMap = Mat::zeros(left.rows, left.cols, CV_8UC1);
    Mat depthMap = Mat::zeros(left.rows, left.cols, CV_8UC1);

    SOS(left, right, maxDisp, disparityMap, depthMap, 2270.780, 177.288);
    imwrite("disparity_gui.png", disparityMap);

    Mat display;
    cvtColor(disparityMap, display, COLOR_GRAY2BGR); 

    Mat displayGt;
    cvtColor(gt, displayGt, COLOR_GRAY2BGR);


    int maxWidth = 300;
    double scale = min(1.0, static_cast<double>(maxWidth) / display.cols);
    resize(display, display, Size(), scale, scale); 
    resize(displayGt, displayGt, Size(), scale, scale);

    wxImage wxImgGt(displayGt.cols, displayGt.rows, displayGt.data, true);
    groundTruthBitmap->SetBitmap(wxBitmap(wxImgGt));
    groundTruthBitmap->Refresh();
   

    wxImage wxImg(display.cols, display.rows, display.data, true);
    disparityBitmap->SetBitmap(wxBitmap(wxImg));
    disparityBitmap->Refresh();
    Layout(); 
    FitInside();
  
    resize(gt, gt, disparityMap.size());

    float score = computeScore(disparityMap, gt);
    scoreText->SetLabel("Score: " + to_string(score * 100).substr(0, 5) + "%");

    Mat errorMap = generateErrorVisualization(disparityMap, gt);

    resize(errorMap, errorMap, Size(), scale, scale);

    imwrite("error_map.png", errorMap);

    cvtColor(errorMap, errorMap, COLOR_BGR2RGB);

    
    

    wxImage wxImgErr(errorMap.cols, errorMap.rows, errorMap.data, true);
    errorBitmap->SetBitmap(wxBitmap(wxImgErr));
    errorBitmap->Refresh();
    Layout();
    FitInside();
   

    panel->Layout();
    panel->FitInside();
    panel->Refresh();
    panel->Update();
    
}