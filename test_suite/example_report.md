# Example Test Report Output

Sample console output from a vision workflow test run.

---

## Console Output

```
================================================================================
VISION WORKFLOW TEST REPORT
================================================================================
Test started at: 2026-01-09 17:48:01
================================================================================

[Test 1] Service available: True (took 0.00s)
[Test 2] Service response: True (2485.69ms)
         Message: Added 2 potential new detections
[Test 3] Detections published: 2 objects
         Latency: 1311.16ms
[Test 4] Detection format validated: 2 objects
         Object 1: flower_pot (93.06%)
           BBox: (575.0, 273.9) 155.0x254.0px
         Object 2: box_pp (62.55%)
           BBox: (400.8, 599.0) 118.0x118.0px
```

## Quick Summary

```
================================================================================
VISION WORKFLOW TEST SUMMARY
================================================================================

────────────────────────────────────────────────────────────────────────────────
QUICK SUMMARY
────────────────────────────────────────────────────────────────────────────────
  - Duration: 15.28 seconds
  - Service Response: 2485.69ms average
  - Detection Latency: 1311.16ms average
  - Objects Detected: 2 objects
    - 1 box_pp (62.55%)
    - 1 flower_pot (93.06%)

Test completed at: 2026-01-09 17:48:16

Service Response Times:
  Average: 2485.69ms
  Min: 2485.69ms
  Max: 2485.69ms

Detection Latencies:
  Average: 1311.16ms
  Min: 1311.16ms
  Max: 1311.16ms
```

## Detailed Detection Results

```
────────────────────────────────────────────────────────────────────────────────
DETECTION RESULTS
────────────────────────────────────────────────────────────────────────────────

Total detections captured: 1

Detection Set #1:
  Timestamp: 2026-01-09 17:48:16.166
  Number of objects: 2

  Object 1:
    Class ID: flower_pot
    Confidence: 93.06%
    Bounding Box:
      Center: (575.0, 273.9)
      Size: 155.0 x 254.0 pixels
      Area: 39370.0 px²

  Object 2:
    Class ID: box_pp
    Confidence: 62.55%
    Bounding Box:
      Center: (400.8, 599.0)
      Size: 118.0 x 118.0 pixels
      Area: 13924.0 px²
```

## Test Results Summary

```
────────────────────────────────────────────────────────────────────────────────
TEST RESULTS
────────────────────────────────────────────────────────────────────────────────
✓ PASSED - Service Availability
  Service became available in 0.00s
✓ PASSED - Trigger Detection Service
  Response time: 2485.69ms, Message: Added 2 potential new detections
✓ PASSED - Detections Published
  Received 2 detections, latency: 1311.16ms
✓ PASSED - Detection Format & Details
  Validated 2 detections with proper format
```
