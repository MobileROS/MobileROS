/*
 * Copyright (c) 2007-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_UPER_OPENTYPE_H_
#define	_UPER_OPENTYPE_H_

#include <per_opentype.h>

#ifdef __cplusplus
extern "C" {
#endif

asn_dec_rval_t uper_open_type_get_e2ap_v2_03(const asn_codec_ctx_t *opt_codec_ctx,
                                  const asn_TYPE_descriptor_t *td,
                                  const asn_per_constraints_t *constraints,
                                  void **sptr, asn_per_data_t *pd);

int uper_open_type_skip_e2ap_v2_03(const asn_codec_ctx_t *opt_codec_ctx,
                        asn_per_data_t *pd);

/*
 * X.691 (2015/08), #11.2
 * Returns -1 if error is encountered. 0 if all OK.
 */
int uper_open_type_put_e2ap_v2_03(const asn_TYPE_descriptor_t *td,
                       const asn_per_constraints_t *constraints,
                       const void *sptr, asn_per_outp_t *po);

#ifdef __cplusplus
}
#endif

#endif	/* _UPER_OPENTYPE_H_ */
