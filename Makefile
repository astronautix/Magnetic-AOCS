install:
	python3.8 -m pip install scikit-build
	python3.8 -m pip install --upgrade ninja
	python3.8 -m pip install -r requirements.txt
	python3.8 -c "import wmm2015"
