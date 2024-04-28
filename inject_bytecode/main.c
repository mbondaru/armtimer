#include "stdio.h"
#include "stdlib.h"
#include "fcntl.h"
#include "stdint.h"
#include "unistd.h"
#include <sys/stat.h>
int main(char argc, char *argv[])
{
   int fd_dest;
   int fd_src;
   int fd_res;
   char c;
   uint32_t i, j, dest_read_num, src_read_num, srcbytesread, destbytesread;
   char srcfilename[100];
   char destfilename[100];
   uint32_t srcsize;
   uint32_t destsize;
   uint32_t dest_offset;
   char destreadbuf[4096];
   char srcreadbuf[4096];
   struct stat deststat, srcstat;

   if(argc != 3)
   {
      printf("Usage ./ib destfile srcfile offset\n");
      return 1;
   }
   if((fd_dest = open(argv[0], O_RDONLY)) < 0)
   {
      perror("Opening destfile");
      exit(1);
   }
   if((fd_src = open(argv[1], O_RDONLY)) < 0)
   {
      perror("Opening srcfile");
      exit(1);
   }
   if(argv[2][1] == 'x')
   {
      dest_offset = ((argv[2][2] - ((argv[2][2] > 0x39) ? 0x41 : 0x30)) << 28) |
     	 ((argv[2][3] - ((argv[2][3] > 0x39) ? 0x41 : 0x30)) << 24) |
         ((argv[2][4] - ((argv[2][4] > 0x39) ? 0x41 : 0x30)) << 20) |
         ((argv[2][5] - ((argv[2][5] > 0x39) ? 0x41 : 0x30)) << 16) |
         ((argv[2][6] - ((argv[2][6] > 0x39) ? 0x41 : 0x30)) << 12) |
         ((argv[2][7] - ((argv[2][7] > 0x39) ? 0x41 : 0x30)) << 8) |
         ((argv[2][8] - ((argv[2][8] > 0x39) ? 0x41 : 0x30)) << 4) |
         ((argv[2][9] - ((argv[2][9] > 0x39) ? 0x41 : 0x30)));
   }
   else
   {
      dest_offset = (uint32_t) atoi(argv[2]);
   }

   stat(argv[0], &deststat);
   stat(argv[1], &srcstat);
   if(dest_offset > deststat.st_size)
   {
      printf("Invalid offset");
      return 1;
   }
   else if(srcstat.st_size == 0)
   {
      printf("Empty srcfile");
      return 1;
   }
   else if(deststat.st_size == 0)
   {
      printf("Empty destfile");
      return 1;
   }
   else
   {
      srcbytesread = 0;
      destbytesread = 0;
      while(destbytesread < dest_offset)
      {
         dest_read_num = read(fd_dest, destreadbuf, 4096);
         for(i = 0; 
	      i < ((dest_read_num + destbytesread) <= dest_offset ? 
		      dest_read_num : (dest_offset - 4096)); 
	      i++)
	 {
            if((c = fputc(destreadbuf[i], stdout)) == EOF)
	    {
               perror("Write error");
	       exit(1);
	    }
	 }
	 destbytesread += dest_read_num;
      } 
      while(srcbytesread < srcstat.st_size)
      {
         src_read_num = read(fd_src, srcreadbuf, 4096);
	 for(j = 0; j < src_read_num; j++)
	 {
            if((c = fputc(srcreadbuf[j], stdout)) == EOF)
	    {
               perror("Write error");
	       exit(1);
	    }
	 }
	 srcbytesread += src_read_num;
      }
      for(; i < dest_read_num; i++)
      {
         if((c = fputc(destreadbuf[i], stdout)) == EOF)
	 {
            perror("Write error");
	    exit(1);
	 }
      }
      while(destbytesread < deststat.st_size)
      {
         dest_read_num = read(fd_dest, destreadbuf, 4096);
         for(i = 0; i < dest_read_num; i++)
	 {
            if((c = fputc(destreadbuf[i], stdout)) == EOF)
	    {
               perror("Write error");
	       exit(1);
	    }
	 }
	 destbytesread += dest_read_num;
      }
   }
   return 0;
}
