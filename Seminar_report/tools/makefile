# ------------------------------------------
#  Common part of the makefile
# ------------------------------------------

# -- set the default values
ifndef TNAME
  TNAME = TeXMain
endif
ifndef TTYPE
  TTYPE = pdf
endif
ifndef FIGDIR
  FIGDIR = figures
endif
ifndef TEXARGS
  TEXARGS = -synctex=-1
endif
ifndef BIB
  BIB  = bibtex
endif
ifndef MDX
  MDX = makeindex 
endif
ifndef DVIPS
  DVIPS =
endif
ifndef REMOVE
  REMOVE = rm -f
endif
ifndef FIG2DEV
  FIG2DEV = fig2dev
endif
ifndef PS2DPF
  PS2PDF = ps2pdf
endif

ifndef BIBTEX
  BIBTEX = 0
endif
ifndef MAKEIDX
  MAKEIDX = 0
endif
ifndef NOMENCL
  NOMENCL = 0
endif

THIRD = 0
ifeq ($(BIBTEX),1)
  THIRD = 1
endif
ifeq ($(MAKEIDX),1)
  THIRD = 1
endif
ifeq ($(NOMENCL),1)
  THIRD = 1
endif


# ------------------------------------------------------------------
#
# -- Graphic files
#
# ------------------------------------------------------------------

# -- all fig-files (the related tex and ps files are listed later)
FIG    = \
$(wildcard ./*/$(FIGDIR)/*.fig) \
$(wildcard ./$(FIGDIR)/*.fig)
SVG    = \
$(wildcard ./*/$(FIGDIR)/*.svg) \
$(wildcard ./$(FIGDIR)/*.svg)

# -- all "stand-alone" figure files (PS, EPS and related PDFs)
#    eps are genrated by gnuplot with terminal epslatex
PSF    = \
$(wildcard ./*/$(FIGDIR)/*.ps) \
$(wildcard ./$(FIGDIR)/*.ps)
EPSF   = \
$(wildcard ./*/$(FIGDIR)/*.eps) \
$(wildcard ./$(FIGDIR)/*.eps)
PDFF   = $(patsubst %.ps,%.pdf,$(PSF))
EPDFF  = $(patsubst %.eps,%.pdf,$(EPSF))


# ------------------------------------------------------------------
#
# -- Our TeX-Files
#
# ------------------------------------------------------------------
TEXF = \
$(wildcard *.sty) \
$(wildcard *.tex) \
$(wildcard ./*/*.tex)\
$(wildcard ./*/$(FIGDIR)/*.tex)
# files that can produce aux-files (for clean)
AUX=\
$(wildcard *.tex) \
$(wildcard ./*/*.tex) \


# ------------------------------------------------------------------
# -- Replace .TEX and .FIG by lowercase versions to guarantee that
#    the subsequent replace rules are matching all files 
#    TODO: is there an "i" for patsubs?
# ------------------------------------------------------------------
TEXFLC1:= $(patsubst %.TEX,%.tex,$(TEXF))
TEXFLC2:= $(patsubst %.STY,%.sty,$(TEXFLC1))
TEXF   := $(TEXFLC2)
AUXLC  := $(patsubst %.TEX,%.tex,$(AUX))
AUX    := $(AUXLC)
FIGLC  := $(patsubst %.FIG,%.fig,$(FIG))
FIG    := $(FIGLC)
SVGLC  := $(patsubst %.SVG,%.svg,$(SVG))
SVG    := $(SVGLC)


# ------------------------------------------------------------------
#
# -- List of ps and tex files generated fromthe fig files
#
# ------------------------------------------------------------------
FPTF = $(patsubst %.fig,%.tex,$(FIG))
FPDF = $(patsubst %.fig,%.pdf,$(FIG))
SPDF = $(patsubst %.svg,%.pdf,$(SVG))

# ------------------------------------------------------------------
#
# -- Here we go...
# -- Rule to make the book 
# -- "all" means to plot figs, TeX, Makeindex, Bibtex, TeX, TeX, DVIPS
# -- For postscript files of PSF a pattern rule exists 
#
# ------------------------------------------------------------------
ifdef DVIPS
  FINAL = $(DVIPS) $(TNAME)
else
  FINAL =
endif

ifeq ($(TTYPE),pdf)
  TEX = pdflatex $(TEXARGS)
else
  TEX = latex $(TEXARGS)
endif

.DELETE_ON_ERROR : $(TNAME).$(TTYPE)

$(TNAME).$(TTYPE) : $(TEXF) $(FPTF) $(FPDF) $(SPDF) $(PSF) $(EPSF) $(PDFF) $(EPDFF)
	$(TEX) $(TNAME)
ifeq ($(BIBTEX),1)
	$(BIB) $(TNAME)
endif
ifeq ($(MAKEIDX),1)
	$(MDX) $(TNAME)
endif
ifeq ($(NOMENCL),1)
	$(MDX) $(TNAME).nlo -s nomencl.ist -o $(TNAME).nls
endif
ifeq ($(THIRD),1)
	$(TEX) $(TNAME)
endif
	$(TEX) $(TNAME)
	$(FINAL)

figures: $(FPTF) $(FPDF) $(SPDF) $(PSF) $(EPSF) $(PDFF) $(EPDFF)


# ------------------------------------------------------------------
# -- Rule to make a ps and tex out of a fig 
# ------------------------------------------------------------------
%.tex : %.fig
	$(FIG2DEV) -L pstex_t -p $1$(patsubst %.tex,%,$@) -F $< $@

%.pdf : %.fig
	$(FIG2DEV) -L pstex -F $< $1$(patsubst %.pdf,%,$@).ps
	$(PS2PDF) -dEPSCrop $1$(patsubst %.pdf,%,$@).ps $@
	$(RM) $1$(patsubst %.pdf,%,$@).ps


# ------------------------------------------------------------------
# -- Rule to make a ps and tex out of a svg
# ------------------------------------------------------------------
%.pdf : %.svg
	inkscape $< --export-pdf=$@  --export-latex --export-area-drawing
	sed -e s!$(notdir $@)!$(basename $@)!g $(basename $@).pdf_tex > $(basename $@).pdf2_tex
	sed -e s/'begingroup'/'begingroup\\fontsize\{10\}\{12\}\\selectfont'/ $(basename $@).pdf2_tex > $1$(patsubst %.pdf,%,$@).tex
	rm $(basename $@).pdf_tex
	rm $(basename $@).pdf2_tex


# ------------------------------------------------------------------
# -- rule to make a pds and tex out of a ps or eps 
# -- this rule also applies for the pdf files generated from 
#    fig files (via ps, cause directly fig->pdf is not possible)
# ------------------------------------------------------------------
%.pdf : %.eps
	$(PS2PDF) -dEPSCrop $< $@

%.pdf : %.ps
	$(PS2PDF) -dEPSCrop $< $@


# ------------------------------------------------------------------
#
# -- Clean up the generated files
#
# ------------------------------------------------------------------
clean : 
	$(REMOVE) $(patsubst %.tex,%.aux,$(AUX))
	$(REMOVE) *.idx 
	$(REMOVE) *.ind
	$(REMOVE) *.ilg
	$(REMOVE) *.blg
	$(REMOVE) *.bbl
	$(REMOVE) *.toc
	$(REMOVE) *.dvi
	$(REMOVE) *.log
	$(REMOVE) *.ps
	$(REMOVE) *.pdf
	$(REMOVE) *.glo
	$(REMOVE) *.gls
	$(REMOVE) *.nlo
	$(REMOVE) *.nls
	$(REMOVE) *~
	$(REMOVE) *.bak
	$(REMOVE) *.out
	$(REMOVE) *.nav
	$(REMOVE) *.snm
	$(REMOVE) *.synctex.gz
	$(REMOVE) core

#EOF 
