/*************************************************************************/ /*!
@File
@Title          Display class external
@Description    Defines DC specific structures which are externally visible
                (i.e. visible to clients of services), but are also required
                within services.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#ifndef DC_EXTERNAL_H
#define DC_EXTERNAL_H

#include "img_types.h"

/*!
 * Maximum size of the display name in DC_DISPLAY_INFO
 */
#define DC_NAME_SIZE	50

/*!
 * Maximum planes supported by DC interface based display drivers.
 */
#define DC_MAX_PLANES		(4)
/*!
 * This contains information about a display.
 * The structure can be queried by services from the display driver via a
 * registered callback.
 *
 *   Structure: #DC_DISPLAY_INFO_TAG
 *   Typedef: ::DC_DISPLAY_INFO
 */
typedef struct DC_DISPLAY_INFO_TAG
{
	IMG_CHAR		szDisplayName[DC_NAME_SIZE];	/*!< Display identifier string */
	IMG_UINT32		ui32MinDisplayPeriod;			/*!< Minimum number of VSync periods */
	IMG_UINT32		ui32MaxDisplayPeriod;			/*!< Maximum number of VSync periods */
	IMG_UINT32		ui32MaxPipes;					/*!< Maximum number of pipes for this display */
	IMG_BOOL		bUnlatchedSupported;			/*!< Can the device be unlatched? */
} DC_DISPLAY_INFO;

/*!
 * When services imports a buffer from the display driver it has to fill
 * this structure to inform services about the buffer properties.
 *
 *   Structure: #DC_BUFFER_IMPORT_INFO_TAG
 *   Typedef: ::DC_BUFFER_IMPORT_INFO
 */
typedef struct DC_BUFFER_IMPORT_INFO_TAG
{
	IMG_UINT32		ePixFormat;			/*!< Enum value of type IMG_PIXFMT for the pixel format */
	IMG_UINT32		ui32BPP;			/*!< Bits per pixel */
	IMG_UINT32		ui32Width[3];		/*!< Width of the different channels (defined by ePixFormat) */
	IMG_UINT32		ui32Height[3];		/*!< Height of the different channels (defined by ePixFormat) */
	IMG_UINT32		ui32ByteStride[3];	/*!< Byte stride of the different channels (defined by ePixFormat) */
	IMG_UINT32		ui32PrivData[3];	/*!< Private data of the display for each of the channels */
} DC_BUFFER_IMPORT_INFO;

/* DC_BUFFER_IMPORT_INFO is passed over the bridge so we need to ensure
 * it has proper size */
static_assert((sizeof(DC_BUFFER_IMPORT_INFO) % 4U) == 0U, "invalid size of DC_BUFFER_IMPORT_INFO");

/*!
 * Configuration details of the frame buffer compression module
 *
 *   Structure: #DC_FBC_CREATE_INFO_TAG
 *   Typedef: ::DC_FBC_CREATE_INFO
 */
typedef struct DC_FBC_CREATE_INFO_TAG
{
	IMG_UINT32		ui32FBCWidth;	/*!< Pixel width that the FBC module is working on */
	IMG_UINT32		ui32FBCHeight;	/*!< Pixel height that the FBC module is working on */
	IMG_UINT32		ui32FBCStride;	/*!< Pixel stride that the FBC module is working on */
	IMG_UINT32		ui32Size;		/*!< Size of the buffer to create */
} DC_FBC_CREATE_INFO;

/*!
 * DC buffer details like frame buffer compression and surface properties
 *
 *   Structure: #DC_BUFFER_CREATE_INFO_TAG
 *   Typedef: ::DC_BUFFER_CREATE_INFO
 */
typedef struct DC_BUFFER_CREATE_INFO_TAG
{
	PVRSRV_SURFACE_INFO		sSurface;	/*!< Surface properties, specified by user */
	IMG_UINT32				ui32BPP;	/*!< Bits per pixel */
	DC_FBC_CREATE_INFO		sFBC;		/*!< Frame buffer compressed specific data */
} DC_BUFFER_CREATE_INFO;

/* DC_BUFFER_CREATE_INFO is passed over the bridge so we need to ensure
 * it has proper size */
static_assert((sizeof(DC_BUFFER_CREATE_INFO) % 4U) == 0U, "invalid size of DC_BUFFER_CREATE_INFO");

#endif /* DC_EXTERNAL_H */
