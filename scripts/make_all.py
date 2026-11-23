"""Reproduce every number, figure and animation: python scripts/make_all.py  (~5 min)."""
import run_experiments, make_figures, make_animation

if __name__ == '__main__':   # required: run_experiments uses multiprocessing
    run_experiments.main()
    make_figures.main()
    make_animation.main()
