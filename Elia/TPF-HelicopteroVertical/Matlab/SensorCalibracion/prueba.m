out = calib_train_from_excel("D:\auto\TPF-HelicopteroVertical\DatosLeidos_04.02.26.xlsx", 1);

opts = struct();
opts.guard = "TFM_CALIB_LN_LUT_H_";
opts.prefix = "TFM";
opts.array_name_beta = "tfm_calib_beta";
opts.array_name_lut  = "tfm_ln1p_lux_lut";

calib_export_header("tfm_calib_ln_lut.h", out, opts);
