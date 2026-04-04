out = calib_train_simple("D:\auto\TPF-HelicopteroVertical\DatosLeidos_04.02.26.xlsx", 1);

opts = struct();
opts.guard  = "TFM_CALIB_SIMPLE_H_";
opts.prefix = "TFM";
opts.fn     = "tfmini_correct_distance_cm_simple";
opts.d_min  = 12;
opts.d_max  = 134;

calib_export_simple_h("tfm_calib_simple.h", out, opts);
