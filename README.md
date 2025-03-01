# MIMO_BFS_Sphere_Decoder

## **Project Overview**
This project involves designing a 4×4 MIMO detector for an 8-PSK system using the Breadth-First Search (BFS) Sphere Decoding algorithm.   
Our design can achieve high performance in terms of Bit Error Rate (BER) versus Signal-to-Noise Ratio (SNR).

### **Key Objectives**
1. Implement BFS-based Sphere Decoding to detect 8-PSK symbols.
2. Simulate ML detection, floating-point BFS, and fixed-point BFS to analyze BER vs. SNR.
3. Develop and validate a hardware design to achieve optimal throughput and minimal resource usage.

---

## **Design Details**

### **Algorithm Implementation**
- **Algorithm Used**: 8-Best Breadth-First Search (BFS).
- **Simulation Setup**:
  - Generated 500 random channel matrices (4x4) with 40 transmitted signals each.
  - SNR range: -4 to 25 dB, in steps of 1 dB.
  - Quantization: 4-bit integer and 6-bit fractional precision.
  - Hardware-accurate fixed-point operations simulated using truncation and two's complement for negative multiplications.

- **Performance Metrics**:
  - BER vs. SNR curves for ML detection, floating-point BFS, and fixed-point BFS.
  - SNR degradation calculated at BER = 10^-4.

### **Hardware Design**

#### **Interface Design**
- **Additional IO Signals**:
  - Input valid
  - Output valid
- **Data Handling**:
  - Outputs only detected symbols (12 bits), with demapper implemented in testbench.

#### **Block Diagram Description**
- **Key Modules**:
  - BFS Decoder: Performs decoding.
  - R-Regs & Z-Regs: Store channel matrix and received signals.
  - K-Best Regs: Store best 8 points per layer.
  - FSM: Controls the circuit operation.
  - Counter: Tracks operation points.
- **Optimization**:
  - Pipeline processing to reduce sorting requirements.
  - Insertions into K-best registers avoid full sorting.

#### **Pipeline Challenges**
1. **Pipeline Cutoff Points**:
   - Based on timing constraints, pipelines were inserted after every 2-3 adders.
2. **Layer Switching**:
   - Counter used to stall pipeline until remaining data from the current layer is processed, ensuring data integrity.

---

## **Simulation and Results**

### **Behavioral Simulation**
- SNR Setting: 10 dB.
- Timing Diagram: Captured and verified for functional correctness.

### **Post-Synthesis Simulation**
- Clock Period: 13 ns.
- Processing Time:
  - First vector detected: 233 cycles.
  - Last vector detected: 2413 cycles.
  - Total processing time: 2834 ns.

### **Resource Utilization**
- LUTs: 1528
- FFs: 867
- Total Area (NA): 2395

### **AT Product Calculation**
- AT = Area × Processing Time = 2395 × 2834 = 6,787,430

---

## **Key Innovations**
1. Efficient resource sharing in BFS decoder.
2. Pipelined design to minimize hardware complexity.
3. Truncation-based fixed-point operations to closely simulate hardware behavior.

---

## **References**
1. A. Burg et al., "VLSI implementation of MIMO detection using the sphere decoding algorithm," IEEE Journal of Solid-State Circuits, vol. 40, no. 7, pp. 1566-1577, 2005.
2. J. Chen et al., "An Intra-Iterative Interference Cancellation Detector for Large-Scale MIMO Communications Based on Convex Optimization," IEEE Transactions on Circuits and Systems I, vol. 63, no. 11, pp. 2062-2072, 2016.


