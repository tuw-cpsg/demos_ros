IMAGES = \
		utility.png \
		cset.png \
		variables.png

all: $(IMAGES)

%.png: %.tex
	./tex2png -D 150 -T -o $@ < $<

clean:
	rm -f $(IMAGES)
