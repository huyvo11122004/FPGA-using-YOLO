YOLOv5n DDR core package cleaned for Libero/kit integration
Files included:
  HDL: conv2d_ddr_core.v
  HDL: conv_layer_ddr.v
  HDL: c3_shortcut_ddr_core.v
  HDL: c3_noadd_ddr_core.v
  HDL: maxpool5x5_ddr_core.v
  HDL: sppf_ddr_core.v
  HDL: upsample_ddr_core.v
  HDL: concat_ddr_core.v
  HDL: detect_raw_conv_ddr_core.v
  HDL: detect_head_ddr_core_full.v
  HDL: yolo_scheduler.v
  HDL: top_yolov5n_ddr.v
  HDL: yolo_ddr_addr_map.vh
  MEM: 120 files

Changes made:
  - Replaced D:/vidu/block1/hdl/... paths with relative filenames.
  - Kept only DDR full-flow core HDL needed by top_yolov5n_ddr.
  - Excluded ModelSim work libs, tb files, ddr sim models, old per-layer top tests.

Not included / still needed for real kit:
  - APB/FIC/UIO wrapper from old project.
  - DDR bridge from top_yolov5n_ddr mem_rd/mem_wr to real DDR/FIC bus.
  - detect_output_writer to store det_valid stream into DDR for CPU.
  - Clock/reset integration and Libero SmartDesign connections.
