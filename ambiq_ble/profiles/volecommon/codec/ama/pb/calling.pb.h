//*****************************************************************************
//
//  calling.pb.h
//! @file
//!
//! @brief Auto-generated (see below).
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_0_0-742e5ac27c of the AmbiqSuite Development Package.
//
//*****************************************************************************
/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.1 at Fri Nov 09 16:58:28 2018. */

#ifndef PB_CALLING_PB_H_INCLUDED
#define PB_CALLING_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _ForwardATCommand
{
    char command[34];
/* @@protoc_insertion_point(struct:ForwardATCommand) */
} ForwardATCommand;

typedef struct _IncomingCall
{
    char caller_number[34];
/* @@protoc_insertion_point(struct:IncomingCall) */
} IncomingCall;

/* Default values for struct fields */

/* Initializer values for message structs */
#define ForwardATCommand_init_default            {""}
#define IncomingCall_init_default                {""}
#define ForwardATCommand_init_zero               {""}
#define IncomingCall_init_zero                   {""}

/* Field tags (for use in manual encoding/decoding) */
#define ForwardATCommand_command_tag             1
#define IncomingCall_caller_number_tag           1

/* Struct field encoding specification for nanopb */
extern const pb_field_t ForwardATCommand_fields[2];
extern const pb_field_t IncomingCall_fields[2];

/* Maximum encoded size of messages (where known) */
#define ForwardATCommand_size                    36
#define IncomingCall_size                        36

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define CALLING_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
