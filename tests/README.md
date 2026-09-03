# Host-side offline validation

This directory contains deterministic, dependency-light fixtures for the MaixCAM marker detector. `generate_dataset.py` writes 640x360 PGM frames and JSONL ground truth. Its six-component geometry matches `marker_geometry.cpp` and the available `Downloads/marker.3mf`: three 30 mm Ls centered at (-25,25), (25,-25), (-25,-25); 14 mm L centered at (33,33); 8 mm squares centered at (14,36) and (36,14). The 3MF confirms a 115x120 mm body. The generator applies rotation, scale/distance, brightness, blur, Gaussian noise, LED stripe/dropout, and black-background variants.

Run without network dependencies:

```powershell
python tests/generate_dataset.py --out tests/fixtures --count 48 --seed 7
python tests/evaluate.py --dataset tests/fixtures
```

`evaluate.py` validates fixture integrity and, when a detector adapter is supplied, evaluates JSONL predictions. PGM is intentional: it is readable by OpenCV (`cv::imread`) and by the standard library, so fixture generation does not require Python OpenCV.

`benchmark.py` drives an external host detector adapter (`--adapter command {image}`) and reports effective FPS plus processing/latency percentiles. The adapter must print one JSON result per image; this maps directly to the documented `DetectionResult` fields.

The CMake harness is optional and only enables a C++ smoke target when OpenCV and the expected public detector headers/sources exist under `code/`. Until then configuration succeeds with a clear SKIP message.
