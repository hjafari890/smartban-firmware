/*
 * ============================================================================
 * edgeai_tinyml.h
 * SmartBAN Embedded Quantized Int8 Neural Network (AAMI EC57 Beat Classifier)
 * Target: TI CC2652R1 (ARM Cortex-M4F @ 48 MHz)
 * ============================================================================
 */

#ifndef EDGEAI_TINYML_H_
#define EDGEAI_TINYML_H_

#include <stdint.h>
#include <stdbool.h>
#include "edgeai_ecg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TINYML_CLASS_N_NORMAL   = 0, /* Normal Sinus Beat (N) */
    TINYML_CLASS_S_SVEB     = 1, /* Supraventricular Ectopic Beat (S) */
    TINYML_CLASS_V_PVC      = 2, /* Premature Ventricular Contraction (V) */
    TINYML_CLASS_F_FUSION   = 3, /* Ventricular Fusion Beat (F) */
    TINYML_CLASS_Q_ARTIFACT = 4  /* Motion Artifact / Unclassifiable (Q) */
} edgeai_tinyml_class_t;

typedef struct {
    edgeai_tinyml_class_t predicted_class;
    char                  class_char;       /* 'N', 'S', 'V', 'F', 'Q' */
    uint8_t               confidence_pct;   /* 0..99% pseudo-probability */
    uint32_t              inference_cycles; /* Exact Cortex-M4F DWT clock cycles */
    uint32_t              inference_us;     /* Microseconds at 48 MHz (cycles / 48) */
    uint32_t              total_beats_classified;
    uint32_t              anomaly_beats_count;
} edgeai_tinyml_result_t;

/**
 * @brief Run fixed-point Int8 neural network inference on the latest confirmed QRS beat.
 * @param ecg_state Pointer to Pan-Tompkins state buffer.
 * @param ecg_res   Pointer to confirmed QRS beat result.
 * @param out       Output classification result and hardware cycle metrics.
 */
void edgeai_tinyml_infer_beat(const edgeai_ecg_state_t *ecg_state,
                              const edgeai_ecg_result_t *ecg_res,
                              edgeai_tinyml_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* EDGEAI_TINYML_H_ */
