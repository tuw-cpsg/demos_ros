IMAGES = \
	state_vector.png \
	observation_vector.png \
	input_vector.png \
	f_vector.png \
	F_matrix.png \
	H_matrix.png \
	predict.png \
	update.png \
	Q_matrix.png \
	R_matrix.png

all: $(IMAGES)

%.png: %.tex
	./tex2png -D 150 -T -o $@ < $<

clean:
	rm -f $(IMAGES)
