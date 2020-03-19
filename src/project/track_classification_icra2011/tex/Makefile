

icra2011.pdf: 	icra2011.tex \
		../plots/stc_table.tex \
		latex_macros.tex supplemental.bib mendeley.bib \
		../plots/testset_purehistogram_frame.pdf \
		static_imagery/anecdote_horizontal.pdf \
		static_imagery/example_objects_blurred.pdf \
		static_imagery/vocii.pdf
	cp mendeley.bib icra2011.bib
	cat supplemental.bib >> icra2011.bib
	pdflatex icra2011.tex
	bibtex icra2011
	pdflatex icra2011.tex
	pdflatex icra2011.tex
	rm *.aux *.bbl *.blg *.dvi *.log *.ps

#	latex icra2011.tex
#	bibtex icra2011
#	latex icra2011.tex
#	latex icra2011.tex
#	dvips icra2011.dvi -o icra2011.ps -t letter -Ppdf -G0
#	ps2pdf icra2011.ps
#	rm *.aux *.bbl *.blg *.dvi *.log *.ps

clean:
	-rm *.aux *.bbl *.blg *.dvi *.log *.ps *.pdf

media/features/montages_created: media/features/ped/* media/features/bike/* media/features/car/*
	cd media/features/ped/ && montage -tile 5x2 CloudProjectionHog* eps2:ped_cph_montage.eps && convert ped_cph_montage.eps ped_cph_montage.pdf
	cd media/features/ped/ && montage -tile 5x2 CloudProjection_* eps2:ped_cp_montage.eps && convert ped_cp_montage.eps ped_cp_montage.pdf
	cd media/features/ped/ && montage -tile 5x1 Spin* eps2:ped_spin_montage.eps && convert ped_spin_montage.eps ped_spin_montage.pdf
	cd media/features/bike/ && montage -tile 5x2 CloudProjectionHog* eps2:bike_cph_montage.eps && convert bike_cph_montage.eps bike_cph_montage.pdf
	cd media/features/bike/ && montage -tile 5x2 CloudProjection_* eps2:bike_cp_montage.eps && convert bike_cp_montage.eps bike_cp_montage.pdf
	cd media/features/bike/ && montage -tile 5x1 Spin* eps2:bike_spin_montage.eps && convert bike_spin_montage.eps bike_spin_montage.pdf
	touch $@

media/training_set/training_examples.pdf:
	-rm $(dir $@)/training_examples.pdf
	cd $(dir $@) && montage -tile 4x4 for_montage/tr* training_examples.pdf

media/test_set/test_examples.pdf:
	-rm $(dir $@)/test_examples.pdf
	cd $(dir $@) && montage -tile 4x4 test* test_examples.pdf

media/test_set/occluded.pdf: media/test_set/occluded/*
	cd $(dir $@) && montage -tile 3x1 occluded/* occluded.pdf

media/test_set/failures.pdf:
	cd $(dir $@)/failures && montage -tile 4x1 car_fn03.png car_fp02.png ped_fp01.png bike_fn01.png ../failures.pdf

media/test_set/successes.pdf:
	cd $(dir $@)/successes && montage -tile 3x4 bg* easy* hard* ../successes.pdf


