/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */



#ifndef INDICATION_EVENT_H
#define INDICATION_EVENT_H

#include <stdint.h>                               // for uint8_t
#include "ap/e2ap_types/common/ric_gen_id.h"  // for ric_gen_id_t
#include "../sm/sm_agent.h"

typedef struct{
  ric_gen_id_t ric_id;
  // Non-owning ptr
  sm_agent_t* sm;
  uint8_t action_id;
  // Unknown type for the E2 Agent.
  // The RAN and the SMs know this type.
  // They will free it.
  void* act_def; // i.e., kpm_act_def_t 
} ind_event_t;

int cmp_ind_event(void const* m0_v, void const* m1_v);

bool eq_ind_event_ric_req_id(const void* value, const void* key);

bool eq_ind_event(const void* value, const void* key);

// void free_ind_event(ind_event_t* src);

#endif

