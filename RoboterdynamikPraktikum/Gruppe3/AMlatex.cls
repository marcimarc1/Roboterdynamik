%###################################################################################################
%--- AMlatex ---------------------------------------------------------------------------------------
%###################################################################################################
% Description:       Auto-generated class supplying all modules and styles of the AMlatex-project
%
% License:           Copyright (c) 2017 Technical University of Munich (https://www.tum.de/) This
%                    work may be distributed and/or modified under the conditions of the LaTeX
%                    Project Public License, either version 1.3 of this license or (at your option)
%                    any later version. The latest version of this license is in http://www.latex-
%                    project.org/lppl.txt and version 1.3 or later is part of all distributions of
%                    LaTeX version 2005/12/01 or later. This work has the LPPL maintenance status
%                    'maintained'. The current maintainer of this work is Philipp Seiwald
%                    (philipp.seiwald@tum.de).
%
% Build-Date:        27.06.2017
% Build-Time:        11:19:24
% Git Branch:        (detachedfroma086188)
% Git Commit Hash:   a086188 (a086188e9ea70bfafeb4abf21d8e52ca103d3556)
%---------------------------------------------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}[1994/06/01]%
\ProvidesClass{AMlatex}[2017/06/27 AMlatex]%
%
%###################################################################################################
%--- OPTIONS ---------------------------------------------------------------------------------------
%###################################################################################################
% List of available options:
% --------------------------
% AMcore (submodule)
% AMfont (submodule)
% AMlang (submodule)
% AMcolor (submodule)
% AMtikz (submodule)
% AMlogo (submodule)
% AMmath (submodule)
% AMlayout (submodule)
% AMbiblio (submodule)
% AMnames (submodule)
% AMutils (submodule)
% AMref (submodule)
%
% AMempty (substyle)
% AMdocument (substyle)
%
% optGerman (option)
% optEnglish (option)
% optRGB (option)
% optCMYK (option)
% optGray (option)
% optMonochrome (option)
% optTikzExternalize (option)
% optBiber (option)
% optBibtex (option)
%
% Main class options: (for enabling/disabling submodules and selecting styles)
% -------------------
\DeclareOption{AMcore}{\def\optAMcore{x}}%
\DeclareOption{AMfont}{\def\optAMfont{x}}%
\DeclareOption{AMlang}{\def\optAMlang{x}}%
\DeclareOption{AMcolor}{\def\optAMcolor{x}}%
\DeclareOption{AMtikz}{\def\optAMtikz{x}}%
\DeclareOption{AMlogo}{\def\optAMlogo{x}}%
\DeclareOption{AMmath}{\def\optAMmath{x}}%
\DeclareOption{AMlayout}{\def\optAMlayout{x}}%
\DeclareOption{AMbiblio}{\def\optAMbiblio{x}}%
\DeclareOption{AMnames}{\def\optAMnames{x}}%
\DeclareOption{AMutils}{\def\optAMutils{x}}%
\DeclareOption{AMref}{\def\optAMref{x}}%
\DeclareOption{AMempty}{\def\optAMempty{x}}%
\DeclareOption{AMdocument}{\def\optAMdocument{x}}%
%
% Submodule and substyle options:
% -------------------------------
\DeclareOption{optGerman}{\def\optGerman{x}}%
\DeclareOption{optEnglish}{\def\optEnglish{x}}%
\DeclareOption{optRGB}{\def\optRGB{x}}%
\DeclareOption{optCMYK}{\def\optCMYK{x}}%
\DeclareOption{optGray}{\def\optGray{x}}%
\DeclareOption{optMonochrome}{\def\optMonochrome{x}}%
\DeclareOption{optTikzExternalize}{\def\optTikzExternalize{x}}%
\DeclareOption{optBiber}{\def\optBiber{x}}%
\DeclareOption{optBibtex}{\def\optBibtex{x}}%
%
\ProcessOptions\relax% End of option declaration
%
% Handling default options:
% -------------------------
% Check number of selected substyles (for 0: auto-select AMempty as substyle (without explicit submodule selection))
%                                    (for 1: select this substyle and load ALL submodules)
%                                    (for >1: throw error)
\newcounter{SelectedSubStyles}% Stores the number of selected substyles
\ifdefined\optAMempty\stepcounter{SelectedSubStyles}\else\fi% Increase the counter by one...
\ifdefined\optAMdocument\stepcounter{SelectedSubStyles}\else\fi% Increase the counter by one...
\ifnum\value{SelectedSubStyles}=0%
%   Load default substyle
    \def\optAMempty{x}%
\else\fi%
\ifnum\value{SelectedSubStyles}=1%
%   Load ALL submodules
    \def\optAMcore{x}%
    \def\optAMfont{x}%
    \def\optAMlang{x}%
    \def\optAMcolor{x}%
    \def\optAMtikz{x}%
    \def\optAMlogo{x}%
    \def\optAMmath{x}%
    \def\optAMlayout{x}%
    \def\optAMbiblio{x}%
    \def\optAMnames{x}%
    \def\optAMutils{x}%
    \def\optAMref{x}%
\else\fi%
\ifnum\value{SelectedSubStyles}>1%
%   Throw error
    \typeout{! Please select only ONE substyle.}%
\else\fi%
%
% Check number of selected submodules (for 0: load ALL submodules)
\newcounter{SelectedSubModules}% Stores the number of selected submodules
\ifdefined\optAMcore\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMfont\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMlang\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMcolor\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMtikz\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMlogo\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMmath\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMlayout\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMbiblio\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMnames\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMutils\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifdefined\optAMref\stepcounter{SelectedSubModules}\else\fi% Increase the counter by one...
\ifnum\value{SelectedSubModules}=0%
%   Load ALL submodules
    \def\optAMcore{x}%
    \def\optAMfont{x}%
    \def\optAMlang{x}%
    \def\optAMcolor{x}%
    \def\optAMtikz{x}%
    \def\optAMlogo{x}%
    \def\optAMmath{x}%
    \def\optAMlayout{x}%
    \def\optAMbiblio{x}%
    \def\optAMnames{x}%
    \def\optAMutils{x}%
    \def\optAMref{x}%
\else\fi%
%
%
%###################################################################################################
%--- BASE CLASS LOADING ----------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMempty\LoadClass[11pt,a4paper,fleqn]{article}\else\fi%
\ifdefined\optAMdocument\LoadClass[11pt,a4paper,fleqn]{article}\else\fi%
%
%
%###################################################################################################
%--- BEGIN MODULE AMcore ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMcore% Start-check if submodule AMcore is enabled
\def\AMcoreLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMcore loaded!.}% Print logging information
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMcore.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================          
\RequirePackage[utf8]{inputenc}% sets input encoding to UTF-8
\RequirePackage{etoolbox}%       for parsing conditional expressions
\RequirePackage{xparse}%         allows advanced command definitions
%
%
%=================
%--- Debugging ---
%=================
% Printing misc information
\def\AMLogInfo#1{\typeout{#1.}} 
%
% Printing warnings
\def\AMLogWarning#1{\typeout{LaTeX Warning: #1 on input line \the\inputlineno.}} 
%
% Printing errors
\def\AMLogError#1{\typeout{! #1.}} 
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMcore.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMcore is enabled
%###################################################################################################
%--- END MODULE AMcore -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMfont ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMfont% Start-check if submodule AMfont is enabled
\def\AMfontLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMfont loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMfont.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{helvet}%      Helvetica font (very similar to Arial and TUMNeueHelvetica)  
\RequirePackage[T1]{fontenc}% Use T1 font encoding (necessary for correct hyphenation of german language)
%
%===================
%--- Definitions ---
%===================                                   
\renewcommand{\familydefault}{\sfdefault}%
\fontfamily{phv}\selectfont%               Set Helvetica as default font
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMfont.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMfont is enabled
%###################################################################################################
%--- END MODULE AMfont -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMlang ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMlang% Start-check if submodule AMlang is enabled
\def\AMlangLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMlang loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMlang.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%===============================
%--- Default Option handling ---
%===============================
% Use german as default, if no language has been selected
\ifdefined\optGerman\else%
\ifdefined\optEnglish\else%
    \def\optGerman{x}%
\fi%
\fi%
%
%
%================
%--- Packages ---
%================  
\ifdefined\optGerman%
    \RequirePackage[english,ngerman]{babel}%
\else\fi%
\ifdefined\optEnglish%
    \RequirePackage[ngerman,english]{babel}%
\else\fi%
\RequirePackage{csquotes}% 
%
%
%================
%--- Commands ---
%================
% Switch language
% ---------------
\newcommand{\AMlangSwitchToGerman}{%
    \def\optGerman{x}%
    \undef\optEnglish%
    \selectlanguage{ngerman}%
}%
\newcommand{\AMlangSwitchToEnglish}{%
    \undef\optGerman%
    \def\optEnglish{x}%
    \selectlanguage{english}%
}%
%
% Auto-select language
% --------------------
\newcommand{\AMGerEng}[2]{\ifdefined\optGerman#1\else\fi\ifdefined\optEnglish#2\else\fi}%
\newcommand{\AMEngGer}[2]{\ifdefined\optEnglish#1\else\fi\ifdefined\optGerman#2\else\fi}%
%
%
%===================
%--- Definitions ---
%===================
% TUM
% ---
\def\AMlangTUM{\AMGerEng{Technische Universität München}{Technical University of Munich}}%
%
% Department
% ----------
\def\AMlangDepartment{\AMGerEng{Fakultät für Maschinenwesen}{Department of Mechanical Engineering}}
%
% Chairs
% ------
\def\AMlangChairAER{\AMGerEng{Lehrstuhl für Aerodynamik und Strömungsmechanik}{Chair of Aerodynamics and Fluid Mechanics}}%
\def\AMlangChairVIB{\AMGerEng{Lehrstuhl für Akustik mobiler Systeme}{Chair of Vibroacoustics of Vehicles and Machines}}%
\def\AMlangChairAM{\AMGerEng{Lehrstuhl für Angewandte Mechanik}{Chair of Applied Mechanics}}%
\def\AMlangChairAPT{\AMGerEng{Lehrstuhl für Anlagen- und Prozesstechnik}{Chair of Plant and Process Technology}}%
\def\AMlangChairAIS{\AMGerEng{Lehrstuhl für Automatisierung und Informationssysteme}{Chair of Automation and Information Systems}}%
\def\AMlangChairIWB{\AMGerEng{Institut für Werkzeugmaschinen und Betriebswissenschaften}{Institute for Machine Tools and Industrial Management}}%
\def\AMlangChairBVT{\AMGerEng{Lehrstuhl für Bioverfahrenstechnik}{Chair of Biochemical Engineering}}%
\def\AMlangChairLCC{\AMGerEng{Lehrstuhl für Carbon Composites}{Chair of Carbon Composites}}%
\def\AMlangChairLES{\AMGerEng{Lehrstuhl für Energiesysteme}{Chair of Energy Systems}}%
\def\AMlangChairLFE{\AMGerEng{Lehrstuhl für Ergonomie}{Chair of Ergonomics}}%
\def\AMlangChairFTM{\AMGerEng{Lehrstuhl für Fahrzeugtechnik}{Chair of Automotive Technology}}%
\def\AMlangChairFSD{\AMGerEng{Lehrstuhl für Flugsystemdynamik}{Chair of Flight System Dynamics}}%
\def\AMlangChairFML{\AMGerEng{Lehrstuhl für Fördertechnik Materialfluss Logistik / Technische Logistik}{Chair of Materials Handling, Material Flow, Logistics / Logistics Engineering}}%
\def\AMlangChairLHT{\AMGerEng{Lehrstuhl für Hubschraubertechnologie}{Chair of Helicopter Technology}}%
\def\AMlangChairPKM{\AMGerEng{Professur für Kontinuumsmechanik}{Associate Professorship of Continuum Mechanics}}%
\def\AMlangChairLLB{\AMGerEng{Lehrstuhl für Leichtbau}{Chair of Lightweight Structures}}%
\def\AMlangChairLLS{\AMGerEng{Lehrstuhl für Luftfahrtsysteme}{Chair of Aircraft Design}}%
\def\AMlangChairFZG{\AMGerEng{Lehrstuhl für Maschinenelemente}{Chair of Machine Elements}}%
\def\AMlangChairMHPC{\AMGerEng{Professur für Mechanik auf Höchstleistungsrechnern}{Associate Professorship of Mechanics and High Performance Computing}}%
\def\AMlangChairLMT{\AMGerEng{Lehrstuhl für Medizintechnik}{Chair of Medical Engineering}}%
\def\AMlangChairLMM{\AMGerEng{Lehrstuhl für Mikrotechnik und Medizingerätetechnik}{Chair of Micro Technology and Medical Device Technology}}%
\def\AMlangChairLNT{\AMGerEng{Lehrstuhl für Nukleartechnik}{Chair of Nuclear Technology}}%
\def\AMlangChairLNM{\AMGerEng{Lehrstuhl für Numerische Mechanik}{Chair of Computational Mechanics}}%
\def\AMlangChairPMW{\AMGerEng{Professur für Plasma-Material-Wechselwirkung}{Associate Professorship of Plasma Material Interaction}}%
\def\AMlangChairPE{\AMGerEng{Lehrstuhl für Produktentwicklung}{Chair of Product Development}}%
\def\AMlangChairPTM{\AMGerEng{Lehrstuhl für Produktion und Technik in der Medienbranche}{Chair of Production and Technology in Media}}%
\def\AMlangChairLFA{\AMGerEng{Lehrstuhl für Raumfahrtantriebe}{Chair of Space Propulsion}}%
\def\AMlangChairLRT{\AMGerEng{Lehrstuhl für Raumfahrttechnik}{Chair of Astronautics}}%
\def\AMlangChairRT{\AMGerEng{Lehrstuhl für Regelungstechnik}{Chair of Automatic Control}}%
\def\AMlangChairSES{\AMGerEng{Professur für Sichere Eingebettete Systeme}{Assistant Professorship of Safe Embedded Systems}}%
\def\AMlangChairSPGM{\AMGerEng{Professur für Sportgeräte und Sportmaterialien}{Associate Professorship of Sport Equipment and Materials}}%
\def\AMlangChairTD{\AMGerEng{Lehrstuhl für Thermodynamik}{Chair of Thermodynamics}}%
\def\AMlangChairTFD{\AMGerEng{Professur für Thermofluiddynamik}{Associate Professorship of Thermo-Fluid Dynamics}}%
\def\AMlangChairUTG{\AMGerEng{Lehrstuhl für Umformtechnik und Gießereiwesen}{Chair of Metal Forming and Casting}}%
\def\AMlangChairLVK{\AMGerEng{Lehrstuhl für Verbrennungskraftmaschinen}{Chair of Internal Combustion Engines}}%
\def\AMlangChairWKM{\AMGerEng{Lehrstuhl für Werkstoffkunde und Werkstoffmechanik}{Chair of Materials Science and Mechanics of Materials}}%
\def\AMlangChairLWE{\AMGerEng{Lehrstuhl für Windenergie}{Chair of Wind Energy}}%
\def\AMlangChairZFP{\AMGerEng{Lehrstuhl für Zerstörungsfreie Prüfung}{Chair of Non-Destructive Testing}}%
%
% Current chair
% -------------
\def\AMlangChair{\AMlangChairAM}%
%
% Roles
% -----
\def\AMlangresearchassistantMale{\AMGerEng{wissenschaftlicher Mitarbeiter}{research assistant}}%
\def\AMlangResearchassistantMale{\AMGerEng{Wissenschaftlicher Mitarbeiter}{Research assistant}}%
\def\AMlangResearchAssistantMale{\AMGerEng{Wissenschaftlicher Mitarbeiter}{Research Assistant}}%
\def\AMlangresearchassistantFemale{\AMGerEng{wissenschaftliche Mitarbeiterin}{research assistant}}%
\def\AMlangResearchassistantFemale{\AMGerEng{Wissenschaftliche Mitarbeiterin}{Research assistant}}%
\def\AMlangResearchAssistantFemale{\AMGerEng{Wissenschaftliche Mitarbeiterin}{Research Assistant}}%
\def\AMlangtenuredprofessorMale{\AMGerEng{Ordinarius}{tenured professor}}%
\def\AMlangTenuredprofessorMale{\AMGerEng{Ordinarius}{Tenured professor}}%
\def\AMlangTenuredProfessorMale{\AMGerEng{Ordinarius}{Tenured Professor}}%
\def\AMlangtenuredprofessorFemale{\AMGerEng{Ordinaria}{tenured professor}}%
\def\AMlangTenuredprofessorFemale{\AMGerEng{Ordinaria}{Tenured professor}}%
\def\AMlangTenuredProfessorFemale{\AMGerEng{Ordinaria}{Tenured Professor}}%
\def\AMlangsupervisor{\AMGerEng{Betreuer}{supervisor}}%
\def\AMlangSupervisor{\AMGerEng{Betreuer}{Supervisor}}%
\def\AMlangsupervision{\AMGerEng{Betreuung}{supervision}}%
\def\AMlangSupervision{\AMGerEng{Betreuung}{Supervision}}%
\def\AMlangexaminer{\AMGerEng{Prüfer}{examiner}}%
\def\AMlangExaminer{\AMGerEng{Prüfer}{Examiner}}%
%
% Student thesis
% --------------
\def\AMlangbachelorsthesis{\AMGerEng{Bachelorarbeit}{bachelor's thesis}}%
\def\AMlangBachelorsthesis{\AMGerEng{Bachelorarbeit}{Bachelor's thesis}}%
\def\AMlangBachelorsThesis{\AMGerEng{Bachelorarbeit}{Bachelor's Thesis}}%
\def\AMlangsemesterthesis{\AMGerEng{Semesterarbeit}{semester thesis}}%
\def\AMlangSemesterthesis{\AMGerEng{Semesterarbeit}{Semester thesis}}%
\def\AMlangSemesterThesis{\AMGerEng{Semesterarbeit}{Semester Thesis}}%
\def\AMlangmastersthesis{\AMGerEng{Masterarbeit}{master's thesis}}%
\def\AMlangMastersthesis{\AMGerEng{Masterarbeit}{Master's thesis}}%
\def\AMlangMastersThesis{\AMGerEng{Masterarbeit}{Master's Thesis}}%
%
% Date and Time
% -------------
\def\AMlangDate{\AMGerEng{Datum}{Date}}%
\newcommand{\AMlangMonth}[1]{%
    \ifdefined\optGerman\ifcase#1\or Januar\or Februar\or M\"arz\or April\or Mai\or Juni\or Juli\or August\or September\or Oktober\or November\or Dezember\fi\else\fi%
    \ifdefined\optEnglish\ifcase#1\or January\or February\or March\or April\or May\or June\or July\or August\or September\or October\or November\or December\fi\else\fi%
}%
\newcommand{\AMlangMonthShort}[1]{%
    \ifdefined\optGerman\ifcase#1\or Jan\or Feb\or M\"ar\or Apr\or Mai\or Jun\or Jul\or Aug\or Sep\or Okt\or Nov\or Dez\fi\else\fi%
    \ifdefined\optEnglish\ifcase#1\or Jan\or Feb\or Mar\or Apr\or May\or Jun\or Jul\or Aug\or Sep\or Oct\or Nov\or Dec\fi\else\fi%
}%
\newcommand{\AMlangmonth}[1]{%
    \ifdefined\optGerman\ifcase#1\or Januar\or Februar\or M\"arz\or April\or Mai\or Juni\or Juli\or August\or September\or Oktober\or November\or Dezember\fi\else\fi%
    \ifdefined\optEnglish\ifcase#1\or january\or february\or march\or april\or may\or june\or july\or august\or september\or october\or november\or december\fi\else\fi%
}%
\newcommand{\AMlangmonthShort}[1]{%
    \ifdefined\optGerman\ifcase#1\or Jan\or Feb\or M\"ar\or Apr\or Mai\or Jun\or Jul\or Aug\or Sep\or Okt\or Nov\or Dez\fi\else\fi%
    \ifdefined\optEnglish\ifcase#1\or jan\or feb\or mar\or apr\or may\or jun\or jul\or aug\or sep\or oct\or nov\or dec\fi\else\fi%
}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMlang.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMlang is enabled
%###################################################################################################
%--- END MODULE AMlang -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMcolor --------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMcolor% Start-check if submodule AMcolor is enabled
\def\AMcolorLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMcolor loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMcolor.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%===============================
%--- Default Option handling ---
%===============================
% Select rgb-model, if no option has been passed
\ifdefined\optRGB\else%
\ifdefined\optCMYK\else%
\ifdefined\optGray\else%
\ifdefined\optMonochrome\else%
    \def\optRGB{}%
\fi%
\fi%
\fi%
\fi%
%
%
%================
%--- Packages ---
%================
% Select color model
\ifdefined\optRGB%
    \RequirePackage[rgb]{xcolor}%
\else\fi%
\ifdefined\optCMYK%
    \RequirePackage[cmyk]{xcolor}%
\else\fi%
\ifdefined\optGray%
    \RequirePackage[gray]{xcolor}%
\else\fi%
\ifdefined\optMonochrome%
    \RequirePackage[monochrome]{xcolor}%
\else\fi%
%
%
%===================
%--- Definitions ---
%=================== 
% TUM colors
\ifdefined\optCMYK%
    % Primary Colors in CMYK
    \definecolor{TUMBlue}{cmyk}{100,43,0,0}%    Pantone 300
    \definecolor{TUMWhite}{cmyk}{0,0,0,0}%
    \definecolor{TUMBlack}{cmyk}{0,0,0,100}%
    % Secondary Colors in CMYK
    \definecolor{TUMBlue1}{cmyk}{100,57,12,70}% Pantone 540
    \definecolor{TUMBlue2}{cmyk}{100,54,4,19}%  Pantone 301
    \definecolor{TUMGray1}{cmyk}{0,0,0,80}%
    \definecolor{TUMGray2}{cmyk}{0,0,0,50}%
    \definecolor{TUMGray3}{cmyk}{0,0,0,20}%
    % Accent Colors in CMYK
    \definecolor{TUMBlue3}{cmyk}{65,19,1,4}%    Pantone 542
    \definecolor{TUMBlue4}{cmyk}{42,9,0,0}%     Pantone 283
    \definecolor{TUMIvory}{cmyk}{3,4,14,8}%
    \definecolor{TUMOrange}{cmyk}{0,65,95,0}%
    \definecolor{TUMGreen}{cmyk}{35,0,100,20}%
\else%
% Primary Colors in RGB
    \definecolor{TUMBlue}{RGB}{0,101,189}%      Pantone 300
    \definecolor{TUMWhite}{RGB}{255,255,255}%
    \definecolor{TUMBlack}{RGB}{0,0,0}%
    % Secondary Colors in RGB
    \definecolor{TUMBlue1}{RGB}{0,51,89}%       Pantone 540
    \definecolor{TUMBlue2}{RGB}{0,82,147}%      Pantone 301
    \definecolor{TUMGray1}{RGB}{51,51,51}%
    \definecolor{TUMGray2}{RGB}{127,127,127}%
    \definecolor{TUMGray3}{RGB}{204,204,204}%
    % Accent Colors in RGB
    \definecolor{TUMBlue3}{RGB}{100,160,200}%   Pantone 542
    \definecolor{TUMBlue4}{RGB}{152,198,234}%   Pantone 283
    \definecolor{TUMIvory}{RGB}{218,215,203}%
    \definecolor{TUMOrange}{RGB}{227,114,34}%
    \definecolor{TUMGreen}{RGB}{162,173,0}%
\fi%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMcolor.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%===================
%--- Definitions ---
%=================== 
% AM colors
\colorlet{AMLogoBlue1}{TUMBlue}%
\colorlet{AMLogoBlue2}{TUMBlue4}%
%
%
\else\fi% End-check if submodule AMcolor is enabled
%###################################################################################################
%--- END MODULE AMcolor ----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMtikz ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMtikz% Start-check if submodule AMtikz is enabled
\def\AMtikzLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMtikz loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMtikz.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{tikz}%
\usetikzlibrary{calc}% 
\usetikzlibrary{external}% 
\ifdefined\optTikzExternalize%
    \tikzexternalize%
\else\fi%
\RequirePackage[framemethod=tikz]{mdframed}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMtikz.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMtikz is enabled
%###################################################################################################
%--- END MODULE AMtikz -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMlogo ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMlogo% Start-check if submodule AMlogo is enabled
\def\AMlogoLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMlogo loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMcolorLoaded\else\typeout{! The package AMcolor has not been loaded.}\fi%
\ifdefined\AMtikzLoaded\else\typeout{! The package AMtikz has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMlogo.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Commands ---
%================
% Logo of the TUM
\newcommand{\AMlogoTUM}[3][!]{%
    \resizebox{#1}{#2}{%
        \tikzset{external/export next=false}%
        \begin{tikzpicture}[y=0.80pt, x=0.80pt, yscale=-1, xscale=1, inner sep=0pt, outer sep=0pt]%
        \begin{scope}[even odd rule,line width=0.800pt]%
          \begin{scope}[fill=#3]%
            \path[fill] (405,0) -- (370,0) -- (370,320) -- (290,320) -- (290,0) -- (255,0) -- (0,0) --
                (0,70) -- (70,70) -- (70,390) -- (140,390) -- (140,70) -- (220,70) -- (220,390) -- 
                (255,390) -- (405,390) -- (440,390) -- (440,70) -- (520,70) -- (520,390) -- (590,390) -- 
                (590,70) --  (670,70) -- (670,390) -- (740,390) -- (740,35) -- (740,0) -- (405,0) -- cycle;
          \end{scope}%
        \end{scope}%
        \end{tikzpicture}%
    }%
}%
%
% Logo of the department of mechanical engineering
\newcommand{\AMlogoDepartment}[3][!]{%
    \resizebox{#1}{#2}{%
        \tikzset{external/export next=false}%
        \begin{tikzpicture}[y=0.80pt, x=0.80pt, yscale=-1, xscale=1, inner sep=0pt, outer sep=0pt]%
            \begin{scope}[even odd rule,line width=0.800pt]%
                \begin{scope}[shift={(-3366.0,-39.0)},fill=#3]%
                    \path[fill] (3848.2448,185.1588) .. controls (3842.0586,169.7349) and
                        (3827.6725,147.4471) .. (3816.1689,135.4551) .. controls (3814.8263,134.0613)
                        and (3783.0794,101.2148) .. (3746.9364,86.3300) -- (3743.6459,94.2852) ..
                        controls (3778.0910,108.4731) and (3809.6141,141.0829) .. (3809.9432,141.4117)
                        .. controls (3820.8019,152.7330) and (3834.3983,173.7980) ..
                        (3840.2291,188.3672) -- (3867.1192,255.3883) -- (3875.1217,252.1799) --
                        (3848.2448,185.1588) -- (3848.2448,185.1588) -- cycle(3475.7058,358.7799) ..
                        controls (3477.0352,360.1737) and (3508.8084,393.0071) .. (3544.9251,407.8787)
                        -- (3548.2024,399.9104) .. controls (3513.7705,385.7487) and
                        (3482.2342,353.1521) .. (3481.9314,352.8365) .. controls (3471.0596,341.4888)
                        and (3457.4763,320.4370) .. (3451.6324,305.8809) -- (3424.7291,238.8204) --
                        (3416.7266,242.0288) -- (3443.6298,309.0762) .. controls (3449.8160,324.5001)
                        and (3464.2022,346.7879) .. (3475.7058,358.7799) -- (3475.7058,358.7799) --
                        cycle(3464.7813,238.7152) .. controls (3462.5042,221.2663) and
                        (3462.4911,203.5413) .. (3464.7813,186.0266) .. controls (3467.2952,166.7895)
                        and (3470.4410,157.8612) .. (3470.4541,157.7692) .. controls
                        (3475.8111,142.9369) and (3488.0913,120.7938) .. (3497.8444,108.3942) --
                        (3542.7270,51.3796) -- (3535.9617,46.0673) -- (3491.0659,103.0819) .. controls
                        (3480.8522,116.0733) and (3467.9665,139.3078) .. (3462.3595,154.8501) ..
                        controls (3462.2147,155.2445) and (3458.8847,164.5936) .. (3456.2391,184.9090)
                        .. controls (3453.8568,203.1731) and (3453.8568,221.6476) ..
                        (3456.2523,239.8329) -- (3464.7813,238.7152) -- (3464.7813,238.7152) --
                        cycle(3636.4277,52.5498) .. controls (3651.3798,48.7366) and
                        (3676.9011,47.3691) .. (3692.1559,49.5650) -- (3764.0339,59.8476) --
                        (3765.2448,51.3270) -- (3693.3931,41.0312) .. controls (3676.9274,38.6907) and
                        (3650.4190,40.1108) .. (3634.2823,44.2001) .. controls (3632.3870,44.6998) and
                        (3587.4122,56.3236) .. (3557.4422,79.3346) -- (3562.6939,86.1590) .. controls
                        (3591.2292,64.2394) and (3635.9802,52.6682) .. (3636.4277,52.5498) --
                        (3636.4277,52.5498) -- cycle(3644.9173,441.2907) -- (3366.0000,441.2907) --
                        (3366.0000,449.9033) -- (3645.9965,449.9033) -- (3645.9965,449.8376) ..
                        controls (3753.8728,449.2459) and (3841.4795,361.4360) .. (3841.4795,253.5080)
                        -- (3832.8583,253.5080) .. controls (3832.8583,357.0442) and
                        (3748.5422,441.2907) .. (3644.9173,441.2907) -- (3644.9173,441.2907) -- cycle;
                \end{scope}%
            \end{scope}%
        \end{tikzpicture}%
    }%
}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMlogo.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Commands ---
%================
% Logo of the chair of Applied Mechanics
\newcommand{\AMlogoAM}[4][!]{%
    \resizebox{#1}{#2}{%
        \tikzset{external/export next=false}%
        \begin{tikzpicture}[y=0.80pt, x=0.80pt, yscale=-1, xscale=1, inner sep=0pt, outer sep=0pt]%
            \begin{scope}[shift={(0,-1016.9291)}]%
                \begin{scope}[cm={{0.85655,0.0,0.0,0.85655,(-0.55087,151.52721)}}]%
                \path[fill=#4,even odd rule] (35.3293,1049.0472) .. controls
                    (33.5002,1048.2521) and (32.2066,1047.3762) .. (31.0238,1046.3603) .. controls
                    (29.2791,1046.8050) and (27.5074,1047.3170) .. (25.7960,1047.8765) .. controls
                    (24.8142,1048.1975) and (23.8529,1048.5338) .. (22.9284,1048.8818) .. controls
                    (23.4872,1049.4012) and (24.0733,1049.8576) .. (24.6898,1050.2399) --
                    (35.3293,1050.2549) -- (35.3293,1049.0470) -- cycle(30.3531,1045.7608) ..
                    controls (29.7790,1045.2283) and (29.2128,1044.6585) .. (28.6020,1044.0441) ..
                    controls (27.5113,1042.9467) and (26.2747,1041.7027) .. (24.5881,1040.2708) ..
                    controls (22.6382,1040.4455) and (20.3088,1040.8781) .. (17.8512,1041.4578) ..
                    controls (18.7335,1043.2433) and (19.6827,1044.9143) .. (20.7362,1046.3653) ..
                    controls (21.2460,1047.0676) and (21.7800,1047.7178) .. (22.3420,1048.3038) ..
                    controls (23.3763,1047.9077) and (24.4581,1047.5261) .. (25.5643,1047.1643) ..
                    controls (27.1304,1046.6523) and (28.7486,1046.1793) .. (30.3531,1045.7608) --
                    cycle(23.8068,1039.5990) .. controls (22.0075,1039.2323) and
                    (20.2217,1038.4864) .. (18.4706,1036.7923) .. controls (18.3232,1036.6499) and
                    (18.1783,1036.5121) .. (18.0351,1036.3792) .. controls (17.6733,1037.3674) and
                    (17.4001,1038.3291) .. (17.2372,1039.2450) .. controls (17.1947,1039.4835) and
                    (17.1596,1039.7188) .. (17.1326,1039.9505) .. controls (17.2591,1040.2251) and
                    (17.3867,1040.4976) .. (17.5157,1040.7679) .. controls (19.7713,1040.2315) and
                    (21.9299,1039.8144) .. (23.8068,1039.5990) -- cycle(15.4106,1034.3103) ..
                    controls (14.1672,1033.5339) and (13.0646,1033.1771) .. (11.9860,1033.1155) ..
                    controls (12.6890,1033.4925) and (13.3409,1034.0357) .. (13.9472,1034.7025) ..
                    controls (14.2895,1035.0788) and (14.6172,1035.4951) .. (14.9315,1035.9430) ..
                    controls (15.0725,1035.4020) and (15.2323,1034.8570) .. (15.4106,1034.3103) --
                    cycle(35.3293,1045.3923) .. controls (34.2309,1045.6046) and
                    (33.0878,1045.8544) .. (31.9256,1046.1360) .. controls (32.8847,1046.9103) and
                    (33.9467,1047.5977) .. (35.3293,1048.2277) -- (35.3293,1045.3923) --
                    cycle(35.3293,1042.2905) .. controls (32.0755,1039.8254) and
                    (28.7192,1040.1943) .. (25.3899,1039.8378) -- (25.3969,1039.9788) .. controls
                    (26.9386,1041.3104) and (28.1001,1042.4786) .. (29.1319,1043.5168) .. controls
                    (29.8636,1044.2528) and (30.5294,1044.9225) .. (31.2225,1045.5390) .. controls
                    (32.6256,1045.1889) and (34.0097,1044.8828) .. (35.3292,1044.6312) --
                    (35.3292,1042.2905) -- cycle(10.0736,1033.2909) .. controls (8.5720,1033.6278)
                    and (6.9846,1034.3861) .. (5.0064,1035.2443) .. controls (7.0806,1036.3381)
                    and (8.4770,1039.6118) .. (9.6562,1042.9573) .. controls (11.1575,1042.4868)
                    and (12.7102,1042.0265) .. (14.2583,1041.6035) .. controls (14.2211,1040.8742)
                    and (14.2413,1040.1220) .. (14.3136,1039.3524) .. controls (14.3880,1038.5615)
                    and (14.5173,1037.7533) .. (14.6949,1036.9337) .. controls (14.2852,1036.2945)
                    and (13.8523,1035.7109) .. (13.3938,1035.2064) .. controls (12.4112,1034.1257)
                    and (11.3110,1033.4103) .. (10.0737,1033.2909) -- cycle(9.9025,1043.6646) ..
                    controls (10.0581,1044.1174) and (10.2104,1044.5688) .. (10.3607,1045.0139) ..
                    controls (11.0668,1047.1066) and (11.7216,1049.0476) .. (12.4168,1050.2225) --
                    (17.9639,1050.2325) .. controls (19.0485,1049.6738) and (20.2745,1049.1260) ..
                    (21.5908,1048.6001) .. controls (21.0813,1048.0488) and (20.5951,1047.4482) ..
                    (20.1296,1046.8069) .. controls (19.0219,1045.2811) and (18.0277,1043.5197) ..
                    (17.1060,1041.6393) -- (17.0837,1041.6493) .. controls (17.0993,1041.8644) and
                    (17.1241,1042.0749) .. (17.1585,1042.2800) .. controls (17.3222,1043.2608) and
                    (17.1655,1044.6558) .. (16.8336,1045.4222) .. controls (16.7096,1045.7082) and
                    (16.5519,1045.9300) .. (16.3631,1046.0512) -- (16.3634,1046.0519) .. controls
                    (16.0941,1046.2248) and (15.8075,1046.2177) .. (15.5187,1045.9719) .. controls
                    (15.2140,1045.7118) and (14.8951,1045.0807) .. (14.5982,1043.9196) .. controls
                    (14.4702,1043.4193) and (14.3781,1042.9020) .. (14.3193,1042.3701) .. controls
                    (12.8329,1042.7786) and (11.3436,1043.2201) .. (9.9026,1043.6716) --
                    cycle(23.3749,1050.2381) .. controls (22.9539,1049.9164) and
                    (22.5482,1049.5617) .. (22.1560,1049.1787) .. controls (21.2833,1049.5213) and
                    (20.4492,1049.8739) .. (19.6689,1050.2328) -- (23.3749,1050.2328) --
                    cycle(4.0171,1035.6659) .. controls (3.4271,1035.9125) and (2.8024,1036.1627)
                    .. (2.1370,1036.4097) -- (2.1370,1045.5532) .. controls (4.0366,1044.8275) and
                    (6.4003,1043.9952) .. (8.9430,1043.1831) .. controls (7.6840,1039.6093) and
                    (6.1909,1036.1517) .. (4.0171,1035.6659) -- cycle(1.3993,1036.6758) --
                    (1.3933,1036.6758) -- (1.3933,1045.8391) -- (1.3993,1045.8391) --
                    (1.3993,1036.6761) -- cycle(11.5636,1050.2211) .. controls (10.9049,1048.9661)
                    and (10.2994,1047.1710) .. (9.6513,1045.2513) .. controls (9.5000,1044.8027)
                    and (9.3462,1044.3470) .. (9.1889,1043.8903) .. controls (6.5318,1044.7382)
                    and (4.0713,1045.6088) .. (2.1370,1046.3550) -- (2.1370,1050.2077) --
                    (11.5636,1050.2217) -- cycle(1.3993,1046.6438) -- (1.3933,1046.6438) --
                    (1.3933,1050.9542) -- (1.7685,1050.9542) -- (1.3997,1050.9539) --
                    (1.3997,1050.5794) -- (1.3997,1046.6413) -- cycle;
                \path[fill=#3,nonzero rule] (36.8235,1034.2149) -- (36.8235,1051.3316) --
                    (36.8235,1051.7065) -- (36.4486,1051.7065) -- (1.0180,1051.7065) --
                    (0.6431,1051.7065) -- (0.6431,1051.3316) -- (0.6431,1032.1435) --
                    (0.6431,1015.9010) -- (0.6431,1015.5261) -- (1.0180,1015.5261) --
                    (25.9438,1015.5261) .. controls (26.0405,1015.7625) and (26.1503,1016.0020) ..
                    (26.2726,1016.2433) -- (26.2726,1016.2763) -- (1.3929,1016.2763) --
                    (1.3929,1032.1439) -- (1.3929,1050.9571) -- (36.0737,1050.9571) --
                    (36.0737,1034.5080) -- (36.0850,1034.5080) -- (36.0967,1034.5080) --
                    (36.1047,1034.5070) -- (36.1087,1034.5060) -- (36.1200,1034.5060) --
                    (36.1313,1034.5060) -- (36.1426,1034.5060) -- (36.1536,1034.5060) --
                    (36.1649,1034.5060) -- (36.1759,1034.5060) -- (36.1862,1034.5060) --
                    (36.1872,1034.5057) -- (36.1982,1034.5057) -- (36.2092,1034.5057) --
                    (36.2202,1034.5057) -- (36.2252,1034.5057) -- (36.2302,1034.5047) --
                    (36.2412,1034.5047) -- (36.2518,1034.5047) -- (36.2734,1034.4947) --
                    (36.2840,1034.4947) -- (36.2946,1034.4947) -- (36.3026,1034.4947) --
                    (36.3056,1034.4940) -- (36.3162,1034.4940) -- (36.3265,1034.4940) --
                    (36.3371,1034.4940) -- (36.3401,1034.4940) -- (36.3471,1034.4940) --
                    (36.3574,1034.4940) -- (36.3677,1034.4940) -- (36.3777,1034.4940) --
                    (36.3781,1034.4940) -- (36.3884,1034.4940) -- (36.3987,1034.4940) --
                    (36.4087,1034.4940) -- (36.4147,1034.4940) -- (36.4197,1034.4940) --
                    (36.4297,1034.4940) -- (36.4397,1034.4940) -- (36.4497,1034.4940) --
                    (36.4507,1034.4933) -- (36.4587,1034.4933) -- (36.4687,1034.4933) --
                    (36.4787,1034.4933) -- (36.4857,1034.4933) -- (36.4877,1034.4933) --
                    (36.4977,1034.4933) -- (36.5077,1034.4933) -- (36.5177,1034.4933) --
                    (36.5217,1034.4933) -- (36.5277,1034.4933) -- (36.5377,1034.4933) --
                    (36.5477,1034.4933) -- (36.5567,1034.4933) -- (36.5567,1034.4929) --
                    (36.5667,1034.4929) -- (36.5757,1034.4929) -- (36.5847,1034.4929) --
                    (36.5897,1034.4929) -- (36.5937,1034.4929) -- (36.6027,1034.4929) --
                    (36.6117,1034.4929) -- (36.6207,1034.4929) -- (36.6227,1034.4919) --
                    (36.6297,1034.4919) -- (36.6387,1034.4919) -- (36.6477,1034.4919) --
                    (36.6547,1034.4919) -- (36.6567,1034.4909) -- (36.6657,1034.4809) --
                    (36.6747,1034.4809) -- (36.6837,1034.4709) -- (36.6867,1034.4709) --
                    (36.6917,1034.4709) -- (36.6997,1034.4609) -- (36.7087,1034.4609) --
                    (36.7167,1034.4509) -- (36.7177,1034.4509) -- (36.7207,1034.4509) --
                    (36.7237,1034.4509) -- (36.7277,1034.4509) -- (36.7307,1034.4509) --
                    (36.7347,1034.4509) -- (36.7387,1034.4509) -- (36.7417,1034.4509) --
                    (36.7437,1034.4509) -- (36.7444,1034.4505) -- (36.7484,1034.4505) --
                    (36.7514,1034.4505) -- (36.7544,1034.4505) -- (36.7564,1034.4495) --
                    (36.7574,1034.4488) -- (36.7614,1034.4488) -- (36.7644,1034.4488) --
                    (36.7674,1034.4488) -- (36.7684,1034.4478) -- (36.7704,1034.4468) --
                    (36.7734,1034.4468) -- (36.7764,1034.4468) -- (36.7794,1034.4468) --
                    (36.7804,1034.4461) -- (36.7824,1034.4451) -- (36.7854,1034.4451) --
                    (36.7884,1034.4451) -- (36.7914,1034.4451) -- (36.7921,1034.4448) --
                    (36.7951,1034.4448) -- (36.7981,1034.4448) -- (36.8011,1034.4448) --
                    (36.8041,1034.4448) -- (36.8045,1034.4448) -- (36.8075,1034.4448) --
                    (36.8115,1034.4448) -- (36.8145,1034.4448) -- (36.8175,1034.4448) --
                    (36.8178,1034.4444) -- (36.8208,1034.4444) -- cycle;
                \path[fill=#3,even odd rule] (17.5603,1035.5274) .. controls
                    (17.2935,1035.3814) and (17.0221,1035.2276) .. (16.7482,1035.0675) --
                    (16.7475,1035.0685) .. controls (16.5257,1034.9388) and (16.3035,1034.8048) ..
                    (16.0820,1034.6677) .. controls (15.5594,1036.2955) and (15.2008,1037.8978) ..
                    (15.0577,1039.4211) -- (15.0577,1039.4221) .. controls (14.9138,1040.9496) and
                    (14.9858,1042.4038) .. (15.3245,1043.7279) .. controls (15.5746,1044.7062) and
                    (15.8010,1045.2108) .. (15.9938,1045.3865) .. controls (16.0413,1045.3365) and
                    (16.0937,1045.2427) .. (16.1476,1045.1190) .. controls (16.4314,1044.4631) and
                    (16.5636,1043.2549) .. (16.4201,1042.3963) .. controls (16.2514,1041.3897) and
                    (16.2907,1040.2845) .. (16.4988,1039.1156) .. controls (16.7018,1037.9750) and
                    (17.0682,1036.7674) .. (17.5603,1035.5269) -- cycle;
                \path[fill=#3,even odd rule] (36.6761,1032.8993) .. controls
                    (36.5616,1032.4033) and (36.0135,1031.8193) .. (34.9207,1031.3438) .. controls
                    (33.7819,1030.8485) and (32.9312,1030.8499) .. (32.4617,1031.1167) .. controls
                    (32.3175,1031.1987) and (32.2154,1031.3059) .. (32.1612,1031.4278) .. controls
                    (32.1081,1031.5472) and (32.0974,1031.6910) .. (32.1336,1031.8473) --
                    (32.1336,1031.8473) .. controls (32.2491,1032.3459) and (32.7997,1032.9337) ..
                    (33.8925,1033.4092) .. controls (35.0232,1033.9014) and (35.8736,1033.8978) ..
                    (36.3452,1033.6299) -- (36.3448,1033.6292) .. controls (36.4901,1033.5462) and
                    (36.5929,1033.4390) .. (36.6471,1033.3178) .. controls (36.7006,1033.1980) and
                    (36.7123,1033.0542) .. (36.6761,1032.8979) -- cycle;
                \path[fill=#3,even odd rule] (36.7633,1031.6592) .. controls
                    (35.5865,1028.7919) and (32.2427,1025.3779) .. (28.1250,1022.8360) .. controls
                    (27.8919,1023.0103) and (27.6598,1023.1924) .. (27.4284,1023.3816) --
                    (27.3515,1023.4726) .. controls (30.2638,1025.5146) and (32.4447,1028.2823) ..
                    (32.4670,1030.3066) .. controls (33.1285,1030.1022) and (34.0764,1030.1610) ..
                    (35.2198,1030.6581) .. controls (35.8941,1030.9515) and (36.4032,1031.2980) ..
                    (36.7632,1031.6595) -- cycle;
                \path[fill=#3,even odd rule] (30.6883,1034.9367) .. controls
                    (30.9292,1034.5909) and (23.1750,1027.8749) .. (22.5344,1029.7989) .. controls
                    (22.5082,1029.8769) and (22.4231,1030.9331) .. (22.3976,1031.3732) .. controls
                    (21.6645,1031.1114) and (20.9204,1030.7921) .. (20.1792,1030.4151) .. controls
                    (15.3036,1027.9373) and (12.3743,1023.9107) .. (13.6386,1021.4233) .. controls
                    (14.6523,1019.4284) and (18.8504,1019.6644) .. (22.7580,1020.9297) .. controls
                    (22.9688,1020.6551) and (23.1810,1020.3851) .. (23.3944,1020.1211) .. controls
                    (16.8353,1017.6078) and (9.9298,1017.5373) .. (8.2294,1020.8843) .. controls
                    (6.2816,1024.7168) and (10.7947,1030.9218) .. (18.3079,1034.7401) .. controls
                    (19.5449,1035.3686) and (20.7865,1035.8934) .. (22.0057,1036.3144) .. controls
                    (21.9667,1036.7562) and (21.8923,1037.4585) .. (21.9316,1037.6211) .. controls
                    (22.4543,1039.7662) and (30.1000,1035.7807) .. (30.6882,1034.9367) -- cycle;
                \path[fill=#3,even odd rule] (18.9776,1028.2041) .. controls
                    (19.2958,1028.4355) and (19.6236,1028.6594) .. (19.9595,1028.8734) --
                    (19.9602,1028.8724) .. controls (20.3000,1029.0886) and (20.6469,1029.2958) ..
                    (20.9991,1029.4914) .. controls (22.3969,1027.4370) and (23.9390,1025.5817) ..
                    (25.5497,1024.0457) .. controls (26.9993,1022.6635) and (28.5081,1021.5356) ..
                    (30.0235,1020.7490) .. controls (29.8524,1020.6080) and (29.6809,1020.4595) ..
                    (29.5108,1020.3040) -- (29.5101,1020.3047) .. controls (29.3216,1020.1325) and
                    (29.1331,1019.9504) .. (28.9464,1019.7587) .. controls (28.6849,1019.5132) and
                    (28.4280,1019.2520) .. (28.1775,1018.9756) -- (28.1768,1018.9763) --
                    (28.1760,1018.9756) -- (28.1720,1018.9756) -- (28.1561,1018.9556) .. controls
                    (27.6877,1018.4369) and (27.2742,1017.8997) .. (26.9195,1017.3601) .. controls
                    (25.5447,1018.6835) and (24.1763,1020.2599) .. (22.8628,1022.0362) .. controls
                    (21.4901,1023.8929) and (20.1766,1025.9700) .. (18.9776,1028.2079) -- cycle;
                \path[fill=#3,even odd rule] (28.7061,1018.4455) -- (28.7068,1018.4462) --
                    (28.7185,1018.4602) .. controls (29.9300,1019.8002) and (31.2998,1020.7623) ..
                    (32.5340,1021.2324) .. controls (33.6870,1021.6718) and (34.6975,1021.6757) ..
                    (35.2907,1021.1394) .. controls (35.3587,1021.0774) and (35.4214,1021.0087) ..
                    (35.4774,1020.9343) .. controls (35.5298,1020.8643) and (35.5770,1020.7876) ..
                    (35.6177,1020.7075) -- (35.7020,1020.4634) -- (35.7013,1020.4630) --
                    (35.7931,1020.1891) -- (35.7924,1020.1888) .. controls (36.2112,1018.9235) and
                    (36.5748,1017.6093) .. (36.9358,1016.3028) .. controls (37.3054,1014.9656) and
                    (37.6739,1013.6337) .. (38.0913,1012.3815) -- (38.0913,1012.3805) .. controls
                    (38.5115,1011.1205) and (38.6132,1010.5574) .. (38.2050,1010.4214) .. controls
                    (37.6643,1010.2410) and (36.4642,1010.3804) .. (34.4296,1010.6155) .. controls
                    (32.7529,1010.8097) and (30.5221,1011.0687) .. (27.5978,1011.2618) .. controls
                    (27.4235,1011.2858) and (27.2594,1011.3318) .. (27.1088,1011.3979) --
                    (27.1092,1011.3982) .. controls (26.9674,1011.4602) and (26.8399,1011.5403) ..
                    (26.7290,1011.6378) -- (26.7297,1011.6385) -- (26.7293,1011.6388) --
                    (26.7148,1011.6508) .. controls (26.1213,1012.1873) and (26.0235,1013.1932) ..
                    (26.3438,1014.3845) .. controls (26.6861,1015.6548) and (27.5007,1017.1097) ..
                    (28.7055,1018.4462) -- (28.7062,1018.4455) -- cycle;
                \end{scope}%
            \end{scope}%
        \end{tikzpicture}%
    }%
}%
%
%
\else\fi% End-check if submodule AMlogo is enabled
%###################################################################################################
%--- END MODULE AMlogo -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMmath ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMmath% Start-check if submodule AMmath is enabled
\def\AMmathLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMmath loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMlangLoaded\else\typeout{! The package AMlang has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMmath.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{amsmath}%
\RequirePackage{amsfonts}%
\RequirePackage{amssymb}%
\RequirePackage{mathtools}%
\RequirePackage{siunitx}%
%
%===================
%--- Definitions ---
%=================== 
% Math operators
% --------------
\DeclareMathOperator{\real}{Re}%
\DeclareMathOperator{\imag}{Im}%
\DeclareMathOperator{\asin}{asin}%
\DeclareMathOperator{\acos}{acos}%
\DeclareMathOperator{\atan}{atan}%
\DeclareMathOperator{\co}{co}%
\DeclareMathOperator{\atanII}{atan2}%
\DeclareMathOperator{\AtanII}{Atan2}%
\DeclareMathOperator{\cosabr}{c}%
\DeclareMathOperator{\sinabr}{s}%
\DeclareMathOperator{\sign}{sign}%
\let\sgn\relax%
\DeclareMathOperator{\sgn}{sgn}%
\DeclareMathOperator{\sat}{sat}%
\DeclareMathOperator{\prox}{prox}%
\DeclareMathOperator{\vprox}{\mathbf{prox}}%
\DeclareMathOperator{\proj}{proj}%
\DeclareMathOperator{\argmin}{arg\,min}%
\DeclareMathOperator{\argmax}{arg\,max}%
\let\div\undefined%
\DeclareMathOperator{\div}{div}%
\DeclareMathOperator{\grad}{grad}%
\DeclareMathOperator{\dif}{d \!}%
\DeclareMathOperator{\Dif}{D \!}%
\DeclareMathOperator{\order}{O}%
%
% Misc
% ----
\newcommand{\foeppl}[1]{\langle #1 \rangle}%
\newcommand{\Abs}[1]{\mathinner{\left\vert#1\right\vert}}%
\newcommand{\abs}[1]{\mathinner{\vert#1\vert}}%
\newcommand{\Norm}[1]{\mathinner{\left\Vert#1\right\Vert}}%
\newcommand{\norm}[1]{\mathinner{\Vert#1\Vert}}%
\newcommand{\e}{\ensuremath{\mathalpha{\mathrm{e}}}}%
\newcommand{\const}{\TUMGerEng{konst}{const}}%
\newcommand{\laplacian}{\Delta}%
%
% Numeral system
% --------------
\newcommand{\hex}[1]{\mathalpha{\mathtt{#1}_\mathrm{H}}}%
\newcommand{\bin}[1]{\mathalpha{\mathtt{#1}_\mathrm{B}}}%
\newcommand{\dec}[1]{\mathalpha{\mathtt{#1}_\mathrm{D}}}%
\newcommand{\reg}[1]{\mathalpha{\mathsf{#1}}}%
\newcommand{\bnot}[1]{\mathalpha{\overline{\mbox{$#1$}}}}%
%
% Sets of numbers
% ---------------
\newcommand{\MR}{\ensuremath{\mathalpha{\mathbb{R}}}}%
\newcommand{\MN}{\ensuremath{\mathalpha{\mathbb{N}}}}%
\newcommand{\MZ}{\ensuremath{\mathalpha{\mathbb{Z}}}}%
\newcommand{\MC}{\ensuremath{\mathalpha{\mathbb{C}}}}%
\newcommand{\MQ}{\ensuremath{\mathalpha{\mathbb{Q}}}}%
\newcommand{\Mone}{\ensuremath{\mathalpha{\mathbb{1}}}}%
%
% Relations
% ---------
\newcommand{\eqhat}{\mathrel{\widehat{=}}}%
\newcommand{\eqexcl}{\stackrel{!}{=}}%
\newcommand{\eqdef}{\stackrel{\mathrm{def.}}{=}}%
\newcommand{\defined}{\coloneqq}%
\newcommand{\rdefined}{\eqqcolon}%
%
% Vectors and matrices
% --------------------
% vectors (main definition)
\let\origvec\vec%
\let\vec\relax%
\newcommand{\vec}[1]{\ensuremath{\boldsymbol{#1}}}%
%
% vectors (latin letters lower case)
\def\vecdeflatinlower#1{\expandafter\def\csname v#1\endcsname{\vec{#1}}}%
\def\vecdeflatinlowerall#1{\ifx#1\vecdeflatinlowerall\else\vecdeflatinlower{#1}\expandafter\vecdeflatinlowerall\fi}%
\vecdeflatinlowerall abcdefghijklmnopqrstuvwxyz\vecdeflatinlowerall%
%
% vectors (latin letters upper case)
\def\vecdeflatinupper#1{\expandafter\def\csname v#1\endcsname{\vec{#1}}}%
\def\vecdeflatinupperall#1{\ifx#1\vecdeflatinupperall\else\vecdeflatinupper{#1}\expandafter\vecdeflatinupperall\fi}%
\vecdeflatinupperall ABCDEFGHIJKLMNOPQRSTUVWXYZ\vecdeflatinupperall%
%
% vectors (greek letters lower case)
\def\vecdefgreeklower#1{\expandafter\def\csname v#1\endcsname{\vec{\csname #1\endcsname}}}%
\def\vecdefgreeklowerall#1{\ifx#1\vecdefgreeklowerall\else\vecdefgreeklower{#1}\expandafter\vecdefgreeklowerall\fi}%
\vecdefgreeklowerall {alpha}{beta}{gamma}{delta}{epsilon}{varepsilon}{zeta}{eta}{theta}{vartheta}{iota}{kappa}{lambda}{mu}{nu}{pi}{varpi}%
{xi}{rho}{varrho}{sigma}{varsigma}{tau}{upsilon}{phi}{varphi}{chi}{psi}{omega}\vecdefgreeklowerall%
%
% vectors (greek letters upper case)
\def\vecdefgreekupper#1{\expandafter\def\csname v#1\endcsname{\vec{\csname #1\endcsname}}}%
\def\vecdefgreekupperall#1{\ifx#1\vecdefgreekupperall\else\vecdefgreekupper{#1}\expandafter\vecdefgreekupperall\fi}%
\vecdefgreekupperall {Gamma}{Delta}{Theta}{Lambda}{Xi}{Pi}{Sigma}{Upsilon}{Phi}{Psi}{Omega}\vecdefgreekupperall%
%
% vectors (misc)
\newcommand{\vnull}{\vec{0}}%
\newcommand{\vzero}{\vec{0}}%
\newcommand{\vone}{\vec{1}}%
%
% matrices
\newcommand*{\mat}[1]{\begin{matrix}#1\end{matrix}}%
\newcommand*{\pmat}[1]{\begin{pmatrix}#1\end{pmatrix}}%
\newcommand*{\bmat}[1]{\begin{bmatrix}#1\end{bmatrix}}%
\newcommand*{\vmat}[1]{\begin{vmatrix}#1\end{vmatrix}}%
\newcommand*{\Vmat}[1]{\begin{Vmatrix}#1\end{Vmatrix}}%
%
% Derivatives
% -----------
\newcommand{\dd}[1]{\dif #1}%
\newcommand{\vdot}[1]{\vec{\dot{#1}}}%
\newcommand{\vddot}[1]{\vec{\ddot{#1}}}%
\newcommand{\tdot}[1]{\tens{\dot{#1}}}%
\newcommand{\tddot}[1]{\tens{\ddot{#1}}}%
\newcommand{\pdiff}[3][]{\frac{\partial^{#1}\,\!#2}{\partial\,\!#3^{#1}}}%
\newcommand{\tpdiff}[3][]{\tfrac{\partial^{#1}\,\!#2}{\partial\,\!#3^{#1}}}%
\newcommand{\Ddiff}[3][]{\frac{D^{#1}\,\!#2}{D\,\!#3^{#1}}}%
\newcommand{\diff}[3][]{\frac{\mathrm{d}^{#1}\,\!#2}{\mathrm{d}\,\!{#3}^{#1}}}%
\newcommand{\tdiff}[3][]{\tfrac{\mathrm{d}^{#1}\,\!#2}{\mathrm{d}\,\!{#3}^{#1}}}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMmath.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMmath is enabled
%###################################################################################################
%--- END MODULE AMmath -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMlayout -------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMlayout% Start-check if submodule AMlayout is enabled
\def\AMlayoutLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMlayout loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMfontLoaded\else\typeout{! The package AMfont has not been loaded.}\fi%
\ifdefined\AMlangLoaded\else\typeout{! The package AMlang has not been loaded.}\fi%
\ifdefined\AMcolorLoaded\else\typeout{! The package AMcolor has not been loaded.}\fi%
\ifdefined\AMtikzLoaded\else\typeout{! The package AMtikz has not been loaded.}\fi%
\ifdefined\AMlogoLoaded\else\typeout{! The package AMlogo has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMlayout.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage[a4paper]{geometry}%
%
%================
%--- Commands ---
%================
% Page setup
% ----------
% Default A4 page
\newcommand{\AMSetLayoutAfourDefault}{%
    \geometry{a4paper,margin=2cm}%
}%
% 
% Absolute Positioning on page
% ----------------------------
% Position in the center of the page
\newcommand{\AMputatCenter}[2][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={($ (\paperwidth/2,\paperheight/2) $)},outer sep=0,inner sep=0,rotate=#1,align=left] at (current page.south west) {#2};%
    \end{tikzpicture}%
}%
%
% Set distance from upper left corner of the page
\newcommand{\AMputatNorthWest}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(#2,-#3)},outer sep=0,inner sep=0,anchor = north west,rotate=#1,align=left] at (current page.north west) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from upper left corner of the page (centered)
\newcommand{\AMputatNorthWestCentered}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(#2,-#3)},outer sep=0,inner sep=0,rotate=#1,align=left] at (current page.north west) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from upper right corner of the page
\newcommand{\AMputatNorthEast}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(-#2,-#3)},outer sep=0,inner sep=0,anchor = north east,rotate=#1,align=left] at (current page.north east) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from upper right corner of the page (centered)
\newcommand{\AMputatNorthEastCentered}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(-#2,-#3)},outer sep=0,inner sep=0,rotate=#1,align=left] at (current page.north east) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from lower left corner of the page
\newcommand{\AMputatSouthWest}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(#2,#3)},outer sep=0,inner sep=0,anchor = south west,rotate=#1,align=left] at (current page.south west) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from lower left corner of the page (centered)
\newcommand{\AMputatSouthWestCentered}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(#2,#3)},outer sep=0,inner sep=0,rotate=#1,align=left] at (current page.south west) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from lower right corner of the page
\newcommand{\AMputatSouthEast}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(-#2,#3)},outer sep=0,inner sep=0,anchor = south east,rotate=#1,align=left] at (current page.south east) {#4};%
    \end{tikzpicture}%
}%
%
% Set distance from lower right corner of the page (centered)
\newcommand{\AMputatSouthEastCentered}[4][0]{%
    \begin{tikzpicture}[remember picture, overlay]%
        \node[shift={(-#2,#3)},outer sep=0,inner sep=0,rotate=#1,align=left] at (current page.south east) {#4};%
    \end{tikzpicture}%
}%
%
%
% Tercets
% -------
% TUM only
\newcommand{\AMTercetTUM}[1][TUMBlue]{%
    \begingroup%
    \color{#1}\fontsize{8}{10.6}\fontfamily{phv}\selectfont%
    \begin{tabular}{@{}l@{}} \\ \\%
        \AMlangTUM%
    \end{tabular}%
    \endgroup%
}%
%
% TUM + department
\newcommand{\AMTercetDepartment}[1][TUMBlue]{%
    \begingroup%
    \color{#1}\fontsize{8}{10.6}\fontfamily{phv}\selectfont%
    \begin{tabular}{@{}l@{}} \\%
        \AMlangDepartment\\%
        \AMlangTUM%
    \end{tabular}%
    \endgroup%
}%
%
% TUM + department + chair
\newcommand{\AMTercetChair}[1][TUMBlue]{%
    \begingroup%
    \color{#1}\fontsize{8}{10.6}\fontfamily{phv}\selectfont%
    \begin{tabular}{@{}l@{}}%
        \AMlangChair\\%
        \AMlangDepartment\\%
        \AMlangTUM%
    \end{tabular}%
    \endgroup%
}%
%
%
% Headers 
% -------
% Corporate design (TUM logo only)
\newcommand{\AMHeaderCDTUMLogoOnly}{%
    \AMputatNorthEast{1cm}{1cm}{\AMlogoTUM{1cm}{TUMBlue}}%
}%
%
% Corporate design (TUM)
\newcommand{\AMHeaderCDTUM}{%
    \AMputatNorthWest{1cm}{1cm}{\AMTercetTUM}%
    \AMputatNorthEast{1cm}{1cm}{\AMlogoTUM{1cm}{TUMBlue}}%
}%
%
% Corporate design (department)
\newcommand{\AMHeaderCDDepartment}{%
    \AMputatNorthWest{1cm}{1cm}{\AMlogoDepartment{1cm}{TUMBlue}}%
    \AMputatNorthWest{2.7cm}{1cm}{\AMTercetDepartment}%
    \AMputatNorthEast{1cm}{1cm}{\AMlogoTUM{1cm}{TUMBlue}}%
}%
%
% Corporate design (chair)
\newcommand{\AMHeaderCDChair}{%
    \AMputatNorthWest{1cm}{1cm}{\AMTercetChair}%
    \AMputatNorthEast{1cm}{1cm}{\AMlogoTUM{1cm}{TUMBlue}}%
}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMlayout.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Commands ---
%================
% Headers 
% -------
% Custom design (chair)
\newcommand{\AMHeaderCustomChair}{%
    \AMputatNorthWest{2cm}{1.35cm}{\AMlogoAM{1.15cm}{AMLogoBlue1}{AMLogoBlue2}}%
    \AMputatNorthWest{3.2cm}{1.5cm}{\AMTercetChair}%
    \AMputatNorthEast{2cm}{1.5cm}{\AMlogoTUM{1cm}{TUMBlue}}%
}%
%
%
% Title pages
% -----------
% Default
\newcommand{\AMTitlePageDefault}[3]{%
    \AMHeaderCustomChair%
    \vskip1.6cm\noindent%
    \ifx\\#1\\\else%    %check, if argument is not empty    
        \textbf{\huge #1}%
    \fi%
    \ifx\\#2\\\else%    %check, if argument is not empty       
        \\[1em]\textbf{\Large \color{TUMBlue} #2}%
    \fi%
    \ifx\\#3\\\else%    %check, if argument is not empty         
        \\[1em]#3%
    \fi%
    \par% 
}%  
%
% Expose student thesis
% Syntax: \AMTitlePageExposeStudentThesis{type of thesis}{provisional title}{author}{matr.nr.}{e-mail}{supervisor}{date}
\newcommand{\AMTitlePageExposeStudentThesis}[7]{%
    \AMHeaderCustomChair
    \begin{flushright}
        \vspace*{1cm}
        Expos\'{e} \AMGerEng{zur}{on} #1\\[0.2cm] 
        \begingroup
        \LARGE \color{TUMBlue} \bfseries
        #2\\[0.2cm]
        \endgroup
        \textbf{\large #3}\\
        \begingroup
        \small
        Matr. Nr.: #4\\
        EMail: #5\\
        \AMlangSupervisor: #6\\[0.2cm]
        \endgroup
        \begingroup
        \large
        \AMlangDate: #7\\
        \endgroup
    \end{flushright}
}%
%
% Expose bachelor's thesis
\newcommand{\AMTitlePageExposeBachelorsThesis}[6]{\AMTitlePageExposeStudentThesis{\AMlangBachelorsThesis}{#1}{#2}{#3}{#4}{#5}{#6}}%
%
% Expose semester thesis
\newcommand{\AMTitlePageExposeSemesterThesis}[6]{\AMTitlePageExposeStudentThesis{\AMlangSemesterThesis}{#1}{#2}{#3}{#4}{#5}{#6}}%
%
% Expose masters's thesis
\newcommand{\AMTitlePageExposeMastersThesis}[6]{\AMTitlePageExposeStudentThesis{\AMlangMastersThesis}{#1}{#2}{#3}{#4}{#5}{#6}}%
%
%
\else\fi% End-check if submodule AMlayout is enabled
%###################################################################################################
%--- END MODULE AMlayout ---------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMbiblio -------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMbiblio% Start-check if submodule AMbiblio is enabled
\def\AMbiblioLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMbiblio loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMlangLoaded\else\typeout{! The package AMlang has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMbiblio.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%=======================
%--- Option handling ---
%=======================
% Check, if a backend has been specified
\ifdefined\optBiber
    \def\BackendSpecified{x}%
\else\fi%
\ifdefined\optBibtex
    \def\BackendSpecified{x}%
\else\fi%
%
%
% Perform rest only, if a backend has been specified 
\ifdefined\BackendSpecified%
%================
%--- Packages ---
%================  
% Select backend
% --------------
\ifdefined\optBiber% Load biblatex package with biber as backend
    \RequirePackage[backend=biber,style=numeric-comp]{biblatex}%
\else\fi%
\ifdefined\optBibtex% Load biblatex package with bibtex as backend
    \RequirePackage[backend=bibtex,style=numeric-comp]{biblatex}%
\else\fi%
%
%
% Set options
% -----------
% Abbreviate first name with inits
\@ifpackagelater{biblatex}{2016/03/13}%
{%
    \ExecuteBibliographyOptions{giveninits=true}% new option
}%
{%
    \ExecuteBibliographyOptions{firstinits=true}% deprecated option
}%
%
% Sorting
\ExecuteBibliographyOptions{sorting=nyt}% Set sorting order
\ExecuteBibliographyOptions{maxnames=99}% Set maximum names of a reference
\DeclareNameAlias{sortname}{last-first}%  Sort by last name
\DeclareNameAlias{default}{last-first}%   Print last name first  
%
%
% End test, if a backend has been specified
\else\fi%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMbiblio.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMbiblio is enabled
%###################################################################################################
%--- END MODULE AMbiblio ---------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMnames --------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMnames% Start-check if submodule AMnames is enabled
\def\AMnamesLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMnames loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMnames.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%==================
%--- Professors ---
%==================
% President
% ---------
\def\AMProfHerrmann{Univ.-Prof.\ Dr.\ Dr.\ h.c.\ mult.\ Wolfgang A. Herrmann}%
%
% Active professors
% -----------------
\def\AMProfAdams{Univ.-Prof.\ Dr.-Ing.\ Nikolaus A. Adams}%
\def\AMProfBengler{Univ.-Prof.\ Dr.\ phil.\ Klaus Bengler}%
\def\AMProfBerensmeier{Univ.-Prof.\ Dr.\ Sonja Berensmeier}%
\def\AMProfBottasso{Univ.-Prof.\ Dr.\ Carlo L. Bottasso}%
\def\AMProfDrechsler{Univ.-Prof.\ Dr.-Ing.\ Klaus Drechsler}%
\def\AMProfFottner{Univ.-Prof.\ Dr.-Ing.\ Johannes Fottner}%
\def\AMProfGee{Univ.-Prof.\ Dr.-Ing.\ Michael W. Gee}%
\def\AMProfGuemmer{Univ.-Prof.\ Dr.-Ing.\ Volker Gümmer}%
\def\AMProfGuenthner{Univ.-Prof.\ Dr.-Ing.\ Willibald A. Günthner}%
\def\AMProfHaidn{Univ.-Prof.\ Dr.-Ing.\ Oskar J. Haidn}%
\def\AMProfHajek{Univ.-Prof.\ Dr.-Ing.\ Manfred Hajek}%
\def\AMProfHolzapfel{Univ.-Prof.\ Dr.-Ing.\ Florian Holzapfel}%
\def\AMProfHornung{Univ.-Prof.\ Dr.-Ing.\ Mirko Hornung}%
\def\AMProfKaltenbach{Univ.-Prof.\ Dr.-Ing.\ Hans-Jakob Kaltenbach}%
\def\AMProfKlein{Univ.-Prof.\ Dr.-Ing.\ Harald Klein}%
\def\AMProfKoutsourelakis{Univ.-Prof.\ Ph.D.\ Phaedon-Stelios Koutsourelakis}%
\def\AMProfKremling{Univ.-Prof.\ Dr.-Ing.\ Andreas Kremling}%
\def\AMProfLieleg{Univ.-Prof.\ Dr.\ Oliver Lieleg}%
\def\AMProfLienkamp{Univ.-Prof.\ Dr.-Ing.\ Markus Lienkamp}%
\def\AMProfLindemann{Univ.-Prof.\ Dr.-Ing.\ Udo Lindemann}%
\def\AMProfLohmann{Univ.-Prof.\ Dr.-Ing.\ Boris Lohmann}%
\def\AMProfLueth{Univ.-Prof.\ Dr.\ rer.\ nat.\ Tim C. Lüth}%
\def\AMProfMacianJuan{Univ.-Prof.\ Dr.\ Rafael Macián-Juan}%
\def\AMProfMarburg{Univ.-Prof.\ Dr.-Ing.\ Steffen Marburg}%
\def\AMProfNeu{Univ.-Prof.\ Dr.\ rer.\ nat.\ Rudolf Neu}%
\def\AMProfPolifke{Univ.-Prof.\ Dr.\ Wolfgang Polifke}%
\def\AMProfProvost{Univ.-Prof.\ Dr.\ Julien Provost}%
\def\AMProfReinhart{Univ.-Prof.\ Dr.-Ing.\ Gunther Reinhart}%
\def\AMProfRixen{Univ.-Prof.\ dr.ir.\ Daniel J. Rixen}%
\def\AMProfSattelmayer{Univ.-Prof.\ Dr.-Ing.\ Thomas F. Sattelmayer}%
\def\AMProfSenner{Univ.-Prof.\ Dr.-Ing.\ Veit Senner}%
\def\AMProfSpannerUlmer{Univ.-Prof.\ Dr.\ Birgit Spanner-Ulmer}%
\def\AMProfSpliethoff{Univ.-Prof.\ Dr.-Ing.\ Hartmut Spliethoff}%
\def\AMProfStahl{Univ.-Prof.\ Dr.-Ing.\ Karsten Stahl}%
\def\AMProfVogelHeuser{Univ.-Prof.\ Dr.-Ing.\ Birgit Vogel-Heuser}%
\def\AMProfVolk{Univ.-Prof.\ Dr.-Ing.\ Wolfram Volk}%
\def\AMProfWachtmeister{Univ.-Prof.\ Dr.\ Georg Wachtmeister}%
\def\AMProfWall{Univ.-Prof.\ Dr.-Ing.\ Wolfgang A. Wall}%
\def\AMProfWalter{Univ.-Prof.\ Dr.\ Ulrich Walter}%
\def\AMProfWerner{Univ.-Prof.\ Dr.\ Ewald Werner}%
\def\AMProfWeusterBotz{Univ.-Prof.\ Dr.-Ing.\ Dirk Weuster-Botz}%
\def\AMProfZaeh{Univ.-Prof.\ Dr.-Ing.\ Michael Zäh}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMnames.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%==================
%--- Professors ---
%================== 
% Retired professors
% ------------------
\def\AMProfUlbrich{Univ.-Prof.\ Dr.-Ing.\ Dr.-Ing.\ habil.\ Heinz Ulbrich,\ i.R.}%
\def\AMProfPfeiffer{Univ.-Prof.\ Dr.-Ing., Dr.-Ing.\,E.h., Dr.-Ing.\,h.c.\,mult.\ Friedrich Pfeiffer, i.R.}%
%
%
%=================
%--- Employees ---
%================= 
% Secretary
% ---------
\def\AMStaffMuellerPhilipp{Manuela Müller-Philipp}%
\def\AMStaffPriller{Daniela Priller}%
%
% Academic director
% -----------------
\def\AMStaffThuemmel{PD\ Dr.-Ing.\ habil.\ Thomas Thümmel}%
%
% Laborartory/Workshop
% --------------------
\def\AMStaffGerer{Simon Gerer}%
\def\AMStaffKoenig{Georg König}%
\def\AMStaffMayr{Georg Mayr}%
%
% Scientific Employees
% --------------------
\def\AMStaffBartl{Dipl.-Ing.\ (Univ.) Andreas Bartl}%
\def\AMStaffBerninger{Tobias Berninger, M.Sc.}%
\def\AMStaffDeimling{Dipl.-Ing.\ Constantin von Deimling}%
\def\AMStaffDewes{Dipl.-Math.\ (Univ.) Eva-Maria Dewes}%
\def\AMStaffEllensohn{Felix Ellensohn, M.Sc.}%
\def\AMStaffGruber{Fabian Gruber, M.Sc.}%
\def\AMStaffHaeussler{Michael Häußler, M.Sc.}%
\def\AMStaffHofmann{Oliver Hofmann, M.Sc.}%
\def\AMStaffKaramoozMahdiabadi{Morteza Karamooz Mahdiabadi, M.Sc.}%
\def\AMStaffLeistner{Dipl.-Ing.\ (Univ.) Michael Leistner}%
\def\AMStaffMaierhofer{Johannes Maierhofer, M.Sc.}%
\def\AMStaffMeyer{Christian Meyer, M.Sc.}%
\def\AMStaffReichart{Ron Reichart, M.Sc.}%
\def\AMStaffSeiwald{Philipp Seiwald, M.Sc.}%
\def\AMStaffSygulla{Felix Sygulla, M.Sc.}%
\def\AMStaffWagner{Christian Wagner, M.Sc.}%
\def\AMStaffWahrmannLockhart{Dipl.-Ing.\ Daniel Wahrmann Lockhart}%
%
%
\else\fi% End-check if submodule AMnames is enabled
%###################################################################################################
%--- END MODULE AMnames ----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMutils --------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMutils% Start-check if submodule AMutils is enabled
\def\AMutilsLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMutils loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMlangLoaded\else\typeout{! The package AMlang has not been loaded.}\fi%
\ifdefined\AMcolorLoaded\else\typeout{! The package AMcolor has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMutils.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{pdfcomment}% Allows PDF-comments and PDF-annotations
\RequirePackage{enumitem}%   Allows to customize enumerations
\RequirePackage{datetime}%   Provides current date and time
\RequirePackage{eurosym}%    Provides the euro symbol
%
%
%================
%--- Commands ---
%================
% Reminders
% ---------
% ToDo (without warning)
\DeclareExpandableDocumentCommand{\AMToDo}{O{red}m}{{\texorpdfstring{\color{#1}\textbf{ToDo:} #2}{}}}%
%
% ToDo (with warning)
\DeclareExpandableDocumentCommand{\AMToDoW}{O{red}m}{\AMToDo[#1]{#2}\AMLogWarning{#2}}%
%
% PDF comments, annotations and highlights
% ----------------------------------------
% PDF annotations (with default color)
\DeclareExpandableDocumentCommand{\AMPDFanno}{O{unknown}m}{\texorpdfstring{\pdfcomment[icon=Note,color=yellow,opacity=0.5,author={#1}]{#2}}{}}%
%
% PDF annotations (with custom color)
\DeclareExpandableDocumentCommand{\AMPDFannoColor}{O{unknown}mm}{\texorpdfstring{\pdfcomment[icon=Note,color=#3,opacity=0.5,author={#1}]{#2}}{}}%
%
% PDF Highlighting
\DeclareExpandableDocumentCommand{\AMPDFhigh}{O{yellow}m}{\texorpdfstring{\pdfmarkupcomment[markup=Highlight,color=#1]{#2}{}}{}}%  
%
% Date and time
% -------------
\newcommand{\AMDate}[3]{\AMGerEng{#1. \AMlangMonth{#2} #3}{\AMlangMonth{#2} #1, #3}}
%
\newcommand{\AMToday}{\AMDate{\the\day}{\the\month}{\the\year}}
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMutils.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMutils is enabled
%###################################################################################################
%--- END MODULE AMutils ----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN MODULE AMref ----------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMref% Start-check if submodule AMref is enabled
\def\AMrefLoaded{x}% Remember, that this submodule has been loaded
\typeout{Submodule AMref loaded!.}% Print logging information
%
% Checking dependencies:
\ifdefined\AMcoreLoaded\else\typeout{! The package AMcore has not been loaded.}\fi%
\ifdefined\AMlangLoaded\else\typeout{! The package AMlang has not been loaded.}\fi%
\ifdefined\AMcolorLoaded\else\typeout{! The package AMcolor has not been loaded.}\fi%
%
%
% >>> CONTENTS OF FILE source/TUM/Modules/TUMref.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{hyperref}%
\hypersetup{%
    %bookmarks=true,%
    linkcolor=TUMBlue,%
    citecolor=TUMBlue,%
    urlcolor=TUMBlue,%
    colorlinks=true}%
\ifdefined\optGerman%
    \RequirePackage[ngerman]{cleveref}%
\else\fi%
\ifdefined\optEnglish%
    \RequirePackage[english]{cleveref}%
\else\fi%
%
%================
%--- Commands ---
%================ 
% Set PDF meta data
\newcommand{\AMSetPDFMetadata}[6]{%
    \hypersetup{pdftitle={#1},pdfsubject={#2},pdfauthor={#3},pdfkeywords={#4},pdfcreator={#5},pdfproducer={#6}}%
}%
%
%
%========================
%--- Execute defaults ---
%========================
\AMSetPDFMetadata{}{}{}{}{TUMlatex project}{}%
%
%
% >>> CONTENTS OF FILE source/AM/Submodules/AMref.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if submodule AMref is enabled
%###################################################################################################
%--- END MODULE AMref ------------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN STYLE AMempty ---------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMempty% Start-check if substyle AMempty is enabled
\def\AMemptyLoaded{x}% Remember, that this substyle has been loaded
\typeout{Substyle AMempty loaded!.}% Print logging information
%
%
% >>> CONTENTS OF FILE source/TUM/Styles/TUMempty.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
% >>> CONTENTS OF FILE source/AM/Substyles/AMempty.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if substyle AMempty is enabled
%###################################################################################################
%--- END STYLE AMempty -----------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- BEGIN STYLE AMdocument ------------------------------------------------------------------------
%###################################################################################################
\ifdefined\optAMdocument% Start-check if substyle AMdocument is enabled
\def\AMdocumentLoaded{x}% Remember, that this substyle has been loaded
\typeout{Substyle AMdocument loaded!.}% Print logging information
%
%
% >>> CONTENTS OF FILE source/TUM/Styles/TUMdocument.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%================
%--- Packages ---
%================
\RequirePackage{caption}%
%
%=============
%--- Style ---
%=============
% Set default layout
\AMSetLayoutAfourDefault%
%
% Paragraph indentation
\parindent0pt%
\parskip0.5em%
%
% Caption style
\captionsetup{labelfont=bf,font=small}%
%
%
% >>> CONTENTS OF FILE source/AM/Substyles/AMdocument.sty: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%
%
\else\fi% End-check if substyle AMdocument is enabled
%###################################################################################################
%--- END STYLE AMdocument --------------------------------------------------------------------------
%###################################################################################################
%
%
%###################################################################################################
%--- DEBUGGING -------------------------------------------------------------------------------------
%###################################################################################################
\newcommand{\AMlatexPrintOptions}{%
\newpage%
\noindent\textbf{Loaded submodules:}\\%
AMcore: \ifdefined\AMcoreLoaded yes\else no\fi\\%
AMfont: \ifdefined\AMfontLoaded yes\else no\fi\\%
AMlang: \ifdefined\AMlangLoaded yes\else no\fi\\%
AMcolor: \ifdefined\AMcolorLoaded yes\else no\fi\\%
AMtikz: \ifdefined\AMtikzLoaded yes\else no\fi\\%
AMlogo: \ifdefined\AMlogoLoaded yes\else no\fi\\%
AMmath: \ifdefined\AMmathLoaded yes\else no\fi\\%
AMlayout: \ifdefined\AMlayoutLoaded yes\else no\fi\\%
AMbiblio: \ifdefined\AMbiblioLoaded yes\else no\fi\\%
AMnames: \ifdefined\AMnamesLoaded yes\else no\fi\\%
AMutils: \ifdefined\AMutilsLoaded yes\else no\fi\\%
AMref: \ifdefined\AMrefLoaded yes\else no\fi\\%
\noindent\textbf{Loaded substyles:}\\%
AMempty: \ifdefined\AMemptyLoaded yes\else no\fi\\%
AMdocument: \ifdefined\AMdocumentLoaded yes\else no\fi\\%
\noindent\textbf{Selected options:}\\%
optGerman: \ifdefined\optGerman yes\else no\fi\\%
optEnglish: \ifdefined\optEnglish yes\else no\fi\\%
optRGB: \ifdefined\optRGB yes\else no\fi\\%
optCMYK: \ifdefined\optCMYK yes\else no\fi\\%
optGray: \ifdefined\optGray yes\else no\fi\\%
optMonochrome: \ifdefined\optMonochrome yes\else no\fi\\%
optTikzExternalize: \ifdefined\optTikzExternalize yes\else no\fi\\%
optBiber: \ifdefined\optBiber yes\else no\fi\\%
optBibtex: \ifdefined\optBibtex yes\else no\fi\\%
}%
%
%
\endinput%