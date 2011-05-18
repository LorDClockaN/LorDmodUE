/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __ARCH_ARM_MACH_MSM_DEBUG_MM_H_
#define __ARCH_ARM_MACH_MSM_DEBUG_MM_H_

/* The below macro removes the directory path name and retains only the
 * file name to avoid long path names in log messages that comes as
 * part of __FILE__ to compiler.
 */
#define __MM_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : \
	__FILE__

#define MM_DBG(fmt, args...) pr_debug("[%s] " fmt,\
		__func__, ##args)

#define MM_INFO(fmt, args...) pr_info("[%s:%s] " fmt,\
	       __MM_FILE__, __func__, ##args)

#define MM_AUD_INFO(fmt, args...) pr_info("[AUD][%s:%s] " fmt,\
			   __MM_FILE__, __func__, ##args)

#define MM_ERR(fmt, args...) pr_err("[%s:%s] " fmt,\
	       __MM_FILE__, __func__, ##args)

#define MM_AUD_ERR(fmt, args...) pr_err("[AUD][%s:%s] " fmt,\
	       __MM_FILE__, __func__, ##args)

#endif /* __ARCH_ARM_MACH_MSM_DEBUG_MM_H_ */
