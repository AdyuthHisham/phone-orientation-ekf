PY ?= python
.PHONY: figures test demo
figures:
	$(PY) scripts/make_all.py
test:
	$(PY) tests/test_filter.py
demo:
	$(PY) src/python/run_demo.py
