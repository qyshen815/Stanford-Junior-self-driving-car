TEX_TURDS := 	ssl_presentation.aux ssl_presentation.log \
		ssl_presentation.nav ssl_presentation.out \
		ssl_presentation.snm ssl_presentation.toc

ssl_presentation.pdf: ssl_presentation.tex
	pdflatex ssl_presentation.tex
	pdflatex ssl_presentation.tex
	rm $(TEX_TURDS)


latex_images: 	../latex/img/ssl_vs_cssl.pdf \
		../latex/img/ssl_induction.pdf \
		../latex/img/frame_induction.pdf \
		../latex/img/ssl_accuracy.pdf \
		../latex/img/ssl_confusion_matrix.pdf \
		../latex/img/baseline_confusion_matrix.pdf \
		../latex/img/accuracy_vs_quantity.pdf

