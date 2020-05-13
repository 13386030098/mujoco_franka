///////////////////////////////////////////////////////////////////////////////
//
//  (C) 2001-2010 Force Dimension
//  All Rights Reserved.
//
//  DHD API 3.2
//
///////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
#include "CTexture.h"
//---------------------------------------------------------------------------

int roundup(int n)
{
    int val = 1;
    while (val < n) val <<= 1;
    return val;
}

//---------------------------------------------------------------------------

void bwtorgba(unsigned char *b,unsigned char *l,int n)
{
    while(n--)
    {
        l[0] = *b;
        l[1] = *b;
        l[2] = *b;
        l[3] = 0xff;
        l += 4; b++;
    }
}

//---------------------------------------------------------------------------

void rgbtorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *l,int n)
{
    while(n--)
    {
        l[0] = r[0];
        l[1] = g[0];
        l[2] = b[0];
        l[3] = 0xff;
        l += 4; r++; g++; b++;
    }
}

//---------------------------------------------------------------------------

void rgbatorgba(unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *a,unsigned char *l,int n)
{
    while(n--)
    {
        l[0] = r[0];
        l[1] = g[0];
        l[2] = b[0];
        l[3] = a[0];
        l += 4; r++; g++; b++; a++;
    }
}

//---------------------------------------------------------------------------

typedef struct _ImageRec
{
    unsigned short imagic;
    unsigned short type;
    unsigned short dim;
    unsigned short xsize, ysize, zsize;
    unsigned int min, max;
    unsigned int wasteBytes;
    char name[80];
    unsigned long colorMap;
    FILE *file;
    unsigned char *tmp, *tmpR, *tmpG, *tmpB;
    unsigned long rleEnd;
    unsigned int *rowStart;
    int *rowSize;
} ImageRec;

//---------------------------------------------------------------------------

static void ConvertShort(unsigned short *array, unsigned int length)
{
    unsigned short b1, b2;
    unsigned char *ptr;

    ptr = (unsigned char *)array;
    while (length--)
    {
        b1 = *ptr++;
        b2 = *ptr++;
        *array++ = (b1 << 8) | (b2);
    }
}

//---------------------------------------------------------------------------

static void ConvertUint(unsigned *array, unsigned int length)
{
    unsigned int b1, b2, b3, b4;
    unsigned char *ptr;

    ptr = (unsigned char *)array;
    while (length--)
    {
        b1 = *ptr++;
        b2 = *ptr++;
        b3 = *ptr++;
        b4 = *ptr++;
        *array++ = (b1 << 24) | (b2 << 16) | (b3 << 8) | (b4);
    }
}

//---------------------------------------------------------------------------

static ImageRec *ImageOpen(const char *fileName)
{
    union
    {
        int testWord;
        char testByte[4];
    } endianTest;
    ImageRec *image;
    int swapFlag;
    int x;

    endianTest.testWord = 1;
    if (endianTest.testByte[0] == 1)
    {
        swapFlag = 1;
    }
    else
    {
        swapFlag = 0;
    }

    image = (ImageRec *)malloc(sizeof(ImageRec));
    if (image == NULL)
    {
        return NULL;
    }

    if ((image->file = fopen(fileName, "rb")) == NULL)
    {
        return NULL;
    }

    fread(image, 1, 12, image->file);

    if (swapFlag)
    {
        ConvertShort(&image->imagic, 6);
    }

    image->tmp = (unsigned char *)malloc(image->xsize*256);
    image->tmpR = (unsigned char *)malloc(image->xsize*256);
    image->tmpG = (unsigned char *)malloc(image->xsize*256);
    image->tmpB = (unsigned char *)malloc(image->xsize*256);

    if (image->tmp == NULL || image->tmpR == NULL || image->tmpG == NULL ||
        image->tmpB == NULL)
    {
        return NULL;
    }

    if ((image->type & 0xFF00) == 0x0100)
    {
        x = image->ysize * image->zsize * (int) sizeof(unsigned);
        image->rowStart = (unsigned *)malloc(x);
        image->rowSize = (int *)malloc(x);

        if (image->rowStart == NULL || image->rowSize == NULL)
        {
            return NULL;
        }

        image->rleEnd = 512 + (2 * x);
        fseek(image->file, 512, SEEK_SET);
        fread(image->rowStart, 1, x, image->file);
        fread(image->rowSize, 1, x, image->file);

        if (swapFlag)
        {
            ConvertUint(image->rowStart, x/(int) sizeof(unsigned));
            ConvertUint((unsigned *)image->rowSize, x/(int) sizeof(int));
        }
    }
    return image;
}

//---------------------------------------------------------------------------

static void ImageClose(ImageRec *image)
{
    fclose(image->file);
    free(image->tmp);
    free(image->tmpR);
    free(image->tmpG);
    free(image->tmpB);
    free(image);
}

//---------------------------------------------------------------------------

static void ImageGetRow(ImageRec *image, unsigned char *buf, int y, int z)
{
    unsigned char *iPtr, *oPtr, pixel;
    int count;

    if ((image->type & 0xFF00) == 0x0100)
    {
        fseek(image->file, (long) image->rowStart[y+z*image->ysize], SEEK_SET);
        fread(image->tmp, 1, (unsigned int)image->rowSize[y+z*image->ysize],
              image->file);

        iPtr = image->tmp;
        oPtr = buf;
        for (;;)
        {
            pixel = *iPtr++;
            count = (int)(pixel & 0x7F);
            if (!count)
            {
                return;
            }
            if (pixel & 0x80)
            {
                while (count--)
                {
                    *oPtr++ = *iPtr++;
                }
            }
            else
            {
                pixel = *iPtr++;
                while (count--)
                {
                    *oPtr++ = pixel;
                }
            }
        }
    }

    else
    {
        fseek(image->file, 512+(y*image->xsize)+(z*image->xsize*image->ysize),
              SEEK_SET);
        fread(buf, 1, image->xsize, image->file);
    }
}

//---------------------------------------------------------------------------

unsigned * read_texture(const char *name, int *width, int *height, int *components)
{
    unsigned *base, *lptr;
    unsigned char *rbuf, *gbuf, *bbuf, *abuf;
    ImageRec *image;
    int y;

    image = ImageOpen(name);

    if(!image)
        return NULL;
    (*width)=image->xsize;
    (*height)=image->ysize;
    (*components)=image->zsize;
    base = (unsigned *)malloc(image->xsize*image->ysize*sizeof(unsigned));
    rbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    gbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    bbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    abuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
    if(!base || !rbuf || !gbuf || !bbuf)
      return NULL;
    lptr = base;
    for(y=0; y<image->ysize; y++)
    {
        if(image->zsize>=4)
        {
            ImageGetRow(image,rbuf,y,0);
            ImageGetRow(image,gbuf,y,1);
            ImageGetRow(image,bbuf,y,2);
            ImageGetRow(image,abuf,y,3);
            rgbatorgba(rbuf,gbuf,bbuf,abuf,(unsigned char *)lptr,image->xsize);
            lptr += image->xsize;
        }

        else if(image->zsize==3)
        {
            ImageGetRow(image,rbuf,y,0);
            ImageGetRow(image,gbuf,y,1);
            ImageGetRow(image,bbuf,y,2);
            rgbtorgba(rbuf,gbuf,bbuf,(unsigned char *)lptr,image->xsize);
            lptr += image->xsize;
        }

        else
        {
            ImageGetRow(image,rbuf,y,0);
            bwtorgba(rbuf,(unsigned char *)lptr,image->xsize);
            lptr += image->xsize;
        }
    }
    ImageClose(image);
    free(rbuf);
    free(gbuf);
    free(bbuf);
    free(abuf);

    return (unsigned *) base;
}

//---------------------------------------------------------------------------

void imgLoad(const char *filenameIn, int borderIn, GLfloat borderColorIn[4], int *wOut, int *hOut, GLubyte ** imgOut)
{
    int border = borderIn;
    int width, height;
    int w, h;
    GLubyte *image, *img, *p;
    int i, j, components;

    image = (GLubyte *) read_texture(filenameIn, &width, &height, &components);
    w = width + 2 * border;
    h = height + 2 * border;
    img = (GLubyte *) calloc(w * h, 4 * sizeof(unsigned char));

    p = img;
    for (j = -border; j < height + border; ++j)
    {
        for (i = -border; i < width + border; ++i)
        {
            if (0 <= j && j <= height - 1 && 0 <= i && i <= width - 1)
            {
                p[0] = image[4 * (j * width + i) + 0];
                p[1] = image[4 * (j * width + i) + 1];
                p[2] = image[4 * (j * width + i) + 2];
                p[3] = 0xff;
            }
            else
            {
                p[0] = (unsigned char)(borderColorIn[0]) * 0xff;
                p[1] = (unsigned char)(borderColorIn[1]) * 0xff;
                p[2] = (unsigned char)(borderColorIn[2]) * 0xff;
                p[3] = (unsigned char)(borderColorIn[3]) * 0xff;
            }
            p += 4;
        }
    }
    free(image);
    *wOut = w;
    *hOut = h;
    *imgOut = img;
}

//---------------------------------------------------------------------------

int create_texture(const char *fname, GLsizei *w,  GLsizei *h, GLsizei *padW, GLsizei *padH, int *comps)
{
    unsigned int *img, *padImg = NULL;
    int y, iw, ih;

    iw = (int)(*w);
    ih = (int)(*h);
    img = read_texture(fname, &iw, &ih, comps);
    if (!img) return -1;
    *w = iw;
    *h = ih;

    /* if width & height are not powers of two, pad image with black */
    if (*w & (*w - 1))
    {
        *padW = roundup(*w);
    }
    else
    {
        *padW = *w;
    }
    if (*h & (*h - 1))
    {
        *padH = roundup(*h);
    }
    else
    {
        *padH = *h;
    }

    if (*padW != *w || *padH != *h)
    {
        printf("rounding %s up...\n", fname);
        padImg = (unsigned int *)malloc(*padW * *padH * sizeof(GLuint));
        if (!padImg)
        {
            fprintf(stderr, "Malloc of %d bytes failed.\n",
            (int)(*padW * *padH * sizeof(GLuint)));
            exit(1);
        }

        memset(padImg, 0, *padW * *padH * sizeof(GLuint));
        for (y = 0; y < *h; y++)
        {
            memcpy(&padImg[y * *padW], &img[y * *w], *w * sizeof(GLuint));
        }
    }


    /* you should use texture objects here if your system supports them... */
    glTexImage2D(GL_TEXTURE_2D, 0, 4, *padW, *padH, 0,
             GL_RGBA, GL_UNSIGNED_BYTE, img);

    free(img);
    if (padImg) free(padImg);

    return 0;
}
