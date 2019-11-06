install:
	git clone https://github.com/neogeo-technologies/pywmm.git
	cd pywmm ; python3 setup.py install && echo 'Successfully installed pywmm' ; cd ..
	rm -rf pywmm
	python3 -m pip install -r requirements.txt
