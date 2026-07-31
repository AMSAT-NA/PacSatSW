/*
 * elfhnd.c
 *
 * ELF file handling
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2014 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation; either version 2 of
 *  the License, or (at your option) any later version.
 *
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this program; if not, write to the Free
 *  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef MY_ELFHND_H
#define MY_ELFHND_H

#include <gelf.h>

struct elfc;

/*
 * Create a temporary file descriptor that will be deleted when
 * closed.  Like tmpfile(), but with an fd.
 * Returns -1 and sets errno on error.
 */
int elfc_tmpfd(void);

/*
 * Copy size byte from infd to outfd.
 * Returns -1 and sets errno on error.
 */
int elfc_copy_fd_range(int outfd, int infd, size_t size);

/*
 * Allocate an ELF object.  This is required to do anything else in
 * this file.
 * Return NULL on error.
 */
struct elfc *elfc_alloc(void);

/*
 * Start working on the elf object, set it up as an elf object of the
 * given type.  Then you can add phdrs, notes, etc. and write it out.
 */
int elfc_setup(struct elfc *e, GElf_Half type);

/*
 * Free all the resources associated with the elf object.
 */
void elfc_free(struct elfc *e);

/*
 * Open an ELF file with the given fd.  This reads in the ehdr, phdrs,
 * and does some basic validation.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_open(struct elfc *e, int fd);

/*
 * Return the last errno for an operation.  Only valid if a function
 * returns -1 and states it uses elfc_get_errno().
 */
int elfc_get_errno(struct elfc *e);

/*
 * Access the file descriptor associated with the elf header.  The get
 * function will return -1 if it is not set.
 */
void elfc_set_fd(struct elfc *e, int fd);
int elfc_get_fd(struct elfc *e);

/*
 * Access the machine type for the file.  This is set by elfc_setup() to
 * the machine it is running on, or EM_NONE if that is not known.
 */
void elfc_setmachine(struct elfc *e, GElf_Half machine);
GElf_Half elfc_getmachine(struct elfc *e);

/*
 * Access the file type for the file.  This is set the setup function
 * or the open function.
 */
void elfc_settype(struct elfc *e, GElf_Half type);
GElf_Half elfc_gettype(struct elfc *e);

/*
 * Return the entry point in the ELF header.
 */
void elfc_setentry(struct elfc *e, GElf_Addr entry);
GElf_Addr elfc_getentry(struct elfc *e);

/*
 * Access the elf class for the file.  This is set by elfc_setup() to
 * the class of the machine it is running on.
 */
void elfc_setclass(struct elfc *e, unsigned char class);
unsigned char elfc_getclass(struct elfc *e);

/*
 * Access the byte encoding for the file.  This is set by elfc_setup() to
 * the endianness of the machine it is running on.
 */
void elfc_setencoding(struct elfc *e, unsigned char encoding);
unsigned char elfc_getencoding(struct elfc *e);

/*
 * Return the total size of the ELF Ehdr for the file.
 */
GElf_Off elfc_ehdr_size(struct elfc *e);

/*
 * Return the size of a single Elf Phdr for the file.
 */
GElf_Off elfc_phdr_size_one(struct elfc *e);

/*
 * Return the size of all the defines Elf Phdrs for the file.
 */
GElf_Off elfc_phdr_size(struct elfc *e);


/*
 * Add a Phdr to the phdr list for the ELF object.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 * Otherwise returns the index of the new phdr.  Note that these
 * indexes can change with later processing (especially writing)
 * so don't assume they stay the same if you have processed the
 * object in other ways.
 */
int elfc_add_phdr(struct elfc *e,
		  GElf_Word type, GElf_Addr vaddr, GElf_Addr paddr,
		  GElf_Xword filesz, GElf_Xword memsz, GElf_Word flags,
		  GElf_Word align);

/*
 * Like elfc_add_phdr(), but inserts the phdr at the given pnum.
 */
int elfc_insert_phdr(struct elfc *e, int pnum,
		     GElf_Word type, GElf_Addr vaddr, GElf_Addr paddr,
		     GElf_Xword filesz, GElf_Xword memsz, GElf_Word flags,
		     GElf_Word align);

/*
 * Delete the given Phdr from the file.  All the header numbers
 * after it will be decremented by one.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_del_phdr(struct elfc *e, int pnum);

/*
 * Set the data handling for the phdr.  This lets you handle the
 * processing of data as it goes out to be written.  You can pass
 * in a data (which is assumed to be a pointer to the actual data)
 * and userdata (which is assumed to be helper data).  Those are
 * not required, though, you can use those for whatever you like.
 * They will just be passed into the given function pointers.
 *
 * The free_func() is called whenever the phdr is destroyed,
 * this is so you can free any data, close file descriptors, etc.
 *
 * The pre_write is called after the Phdrs are processed to find the
 * offsets but before they are written.  The file will not be modified
 * at this time.  This can be used to store off old data, modify the
 * offsets, etc.  You can modify the phdrs at this point, since they
 * are not written yet.
 * 
 * The do_write function should perform the output to the given fd.  The
 * fd will be at the p_offset of the phdr, so you don't have to seek
 * before writing.
 *
 * The post_write function is called after the processing is complete.
 * If the pre_write function is called, the post_write function will
 * be called, even if an error occurs.  This way you can clean things
 * up reliably.
 *
 * The get_data function is called if something is trying to fetch
 * data from the section.  The return results go in odata, so be careful
 * about that, don't use "data".
 *
 * The set_data function is called if something is trying to write
 * data to the section.  The output data comes from idata, so be careful
 * about that.
 *
 * If these functions are NULL, they will not be called.  All these
 * functions should return -1 on error, 0 on success, and set errno
 * on an error.

 * On the headers read in from a file, these functions are
 * automatically set to functions that will save off the old data to a
 * temporary file (pre write), and write the data back into the main
 * file (for do write).
 *
 * elfc_phdr_block_do_write() and elfc_gen_phdr_free() are convenience
 * functions that let you provide a block of data in "data" that is
 * malloced.  That block will be written out.  The size of the block
 * is set by phdr->p_filesz, and the free function will free it.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_set_phdr_data(struct elfc *e, int pnum, void *data,
		       void (*free_func)(struct elfc *e, void *data,
					 void *userdata),
		       int (*pre_write)(struct elfc *e, GElf_Phdr *phdr,
					void *data, void *userdata),
		       int (*do_write)(struct elfc *e, int fd, GElf_Phdr *phdr,
				       void *data, void *userdata),
		       void (*post_write)(struct elfc *e, GElf_Phdr *phdr,
					  void *data, void *userdata),
		       int (*get_data)(struct elfc *e, GElf_Phdr *phdr,
				       void *data,
				       GElf_Off off, void *idata, size_t len,
				       void *userdata),
		       int (*set_data)(struct elfc *e, GElf_Phdr *phdr,
				       void *data, GElf_Off off,
				       const void *idata, size_t len,
				       void *userdata),
		       void *userdata);

/*
 * Convenience functions for elfc_set_phdr_data().  Pass in the free
 * function for free_func and the do_write function for do_write.  See
 * elfc_set_phdr_data() for more details on this.
 *
 * The free func will free data and userdata if not NULL.
 */
void elfc_gen_phdr_free(struct elfc *e, void *data, void *userdata);
int elfc_phdr_block_do_write(struct elfc *e, int fd, GElf_Phdr *phdr,
			     void *data, void *userdata);
int elfc_phdr_block_get_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			     GElf_Off off, void *odata, size_t len,
			     void *userdata);
int elfc_phdr_block_set_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			     GElf_Off off, const void *idata, size_t len,
			     void *userdata);

/*
 * Get a copy of an actual phdr. 
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_get_phdr(struct elfc *e, int pnum, GElf_Phdr *hdr);

/*
 * Get/set the p_offset value for a phdr.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_get_phdr_offset(struct elfc *e, int pnum, GElf_Off *off);
int elfc_set_phdr_offset(struct elfc *e, int pnum, GElf_Off offset);

/*
 * Read from the given offset in the program section pnum.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_phdr_read(struct elfc *e, int pnum, GElf_Off off,
		   void *odata, size_t len);

/*
 * Write to the given offset in the program section pnum.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_phdr_write(struct elfc *e, int pnum, GElf_Off off,
		    const void *odata, size_t len);
  
/*
 * Return the number of phdrs in the ELF object.
 */
int elfc_get_num_phdrs(struct elfc *e);

/*
 * Return the size of a single Elf Shdr for the file.
 */
GElf_Off elfc_shdr_size_one(struct elfc *e);

/*
 * Return the size of all the defines Elf Shdrs for the file.
 */
GElf_Off elfc_shdr_size(struct elfc *e);


/*
 * Add a Shdr to the shdr list for the ELF object.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 * Otherwise returns the index of the new shdr.  Note that these
 * indexes can change with later processing (especially writing)
 * so don't assume they stay the same if you have processed the
 * object in other ways.
 */
int elfc_add_shdr(struct elfc *e,
		  GElf_Word name, GElf_Word type, GElf_Xword flags,
		  GElf_Addr addr, GElf_Off offset, GElf_Xword size,
		  GElf_Word link, GElf_Word info, GElf_Xword addralign,
		  GElf_Xword entsize);
  
/*
 * Like elfc_add_shdr(), but inserts the shdr at the given pnum.
 */
int elfc_insert_shdr(struct elfc *e, int pnum,
		     GElf_Word name, GElf_Word type, GElf_Xword flags,
		     GElf_Addr addr, GElf_Off offset, GElf_Xword size,
		     GElf_Word link, GElf_Word info, GElf_Xword addralign,
		     GElf_Xword entsize);

/*
 * Delete the given Shdr from the file.  All the header numbers
 * after it will be decremented by one.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_del_shdr(struct elfc *e, int pnum);

/*
 * Set the data handling for the shdr.  This lets you handle the
 * processing of data as it goes out to be written.  You can pass
 * in a data (which is assumed to be a pointer to the actual data)
 * and userdata (which is assumed to be helper data).  Those are
 * not required, though, you can use those for whatever you like.
 * They will just be passed into the given function pointers.
 *
 * The free_func() is called whenever the shdr is destroyed,
 * this is so you can free any data, close file descriptors, etc.
 *
 * The pre_write is called after the Shdrs are processed to find the
 * offsets but before they are written.  The file will not be modified
 * at this time.  This can be used to store off old data, modify the
 * offsets, etc.  You can modify the shdrs at this point, since they
 * are not written yet.
 * 
 * The do_write function should perform the output to the given fd.  The
 * fd will be at the p_offset of the shdr, so you don't have to seek
 * before writing.
 *
 * The post_write function is called after the processing is complete.
 * If the pre_write function is called, the post_write function will
 * be called, even if an error occurs.  This way you can clean things
 * up reliably.
 *
 * The get_data function is called if something is trying to fetch
 * data from the section.  The return results go in odata, so be careful
 * about that, don't use "data".
 *
 * The set_data function is called if something is trying to write
 * data to the section.  The output data comes from idata, so be careful
 * about that.
 *
 * If these functions are NULL, they will not be called.  All these
 * functions should return -1 on error, 0 on success, and set errno
 * on an error.

 * On the headers read in from a file, these functions are
 * automatically set to functions that will save off the old data to a
 * temporary file (pre write), and write the data back into the main
 * file (for do write).
 *
 * elfc_shdr_block_do_write() and elfc_gen_shdr_free() are convenience
 * functions that let you provide a block of data in "data" that is
 * malloced.  That block will be written out.  The size of the block
 * is set by shdr->p_filesz, and the free function will free it.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_set_shdr_data(struct elfc *e, int pnum, void *data,
		       void (*free_func)(struct elfc *e, void *data,
					 void *userdata),
		       int (*pre_write)(struct elfc *e, GElf_Shdr *shdr,
					void *data, void *userdata),
		       int (*do_write)(struct elfc *e, int fd, GElf_Shdr *shdr,
				       void *data, void *userdata),
		       void (*post_write)(struct elfc *e, GElf_Shdr *shdr,
					  void *data, void *userdata),
		       int (*get_data)(struct elfc *e, GElf_Shdr *shdr,
				       void *data,
				       GElf_Off off, void *idata, size_t len,
				       void *userdata),
		       int (*set_data)(struct elfc *e, GElf_Shdr *shdr,
				       void *data, GElf_Off off,
				       const void *idata, size_t len,
				       void *userdata),
		       void *userdata);

/*
 * Convenience functions for elfc_set_shdr_data().  Pass in the free
 * function for free_func and the do_write function for do_write.  See
 * elfc_set_shdr_data() for more details on this.
 *
 * The free func will free data and userdata if not NULL.
 */
void elfc_gen_shdr_free(struct elfc *e, void *data, void *userdata);
int elfc_shdr_block_do_write(struct elfc *e, int fd, GElf_Shdr *shdr,
			     void *data, void *userdata);
int elfc_shdr_block_get_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			     GElf_Off off, void *odata, size_t len,
			     void *userdata);
int elfc_shdr_block_set_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			     GElf_Off off, const void *idata, size_t len,
			     void *userdata);

/*
 * Get a copy of an actual shdr. 
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_get_shdr(struct elfc *e, int pnum, GElf_Shdr *hdr);

/*
 * Get/set the p_offset value for a shdr.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_get_shdr_offset(struct elfc *e, int pnum, GElf_Off *off);
int elfc_set_shdr_offset(struct elfc *e, int pnum, GElf_Off offset);

/*
 * Read from the given offset in the program section pnum.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_shdr_read(struct elfc *e, int pnum, GElf_Off off,
		   void *odata, size_t len);

/*
 * Write to the given offset in the program section pnum.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_shdr_write(struct elfc *e, int pnum, GElf_Off off,
		    const void *odata, size_t len);
  
/*
 * Return the number of shdrs in the ELF object.
 */
int elfc_get_num_shdrs(struct elfc *e);

/*
 * Add an ELF note to the object.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_add_note(struct elfc *e, Elf32_Word type,
		  const char *name, int namelen,
		  const void *data, int datalen);

/*
 * Get the total size all the notes will take in the file.  When
 * written this will be the first phdr
 */
GElf_Off elfc_notes_size(struct elfc *e);

/*
 * Return the number of notes in the object.
 */
int elfc_get_num_notes(struct elfc *e);

/*
 * Return the data for the given note.  The pointers are pointers
 * into the actual internal data, but you shouldn't change them.
 * The name is '\0' terminated for convenience, but the termination
 * is after namelen.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_get_note(struct elfc *e, int index,
		  GElf_Word *type,
		  const char **name, size_t *namelen,
		  const void **data, size_t *datalen);

/*
 * Set the data for a note.  If set_type is true, then the type is set.
 * name and namelen are only set if name is non-NULL, data and datalen
 * are only set if data is non-NULL;
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_set_note_data(struct elfc *e, int index,
		       int set_type, GElf_Word type,
		       const char *name, size_t namelen,
		       const void *data, size_t datalen);

/*
 * Delete the given note.
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_del_note(struct elfc *e, int index);

/*
 * Return the total size of all the headers.
 */
GElf_Off elfc_headers_size(struct elfc *e);

/*
 * Return the place where actual data will start to be written for the
 * file.  This is after all headers and notes.
 */
GElf_Off elfc_data_offset_start(struct elfc *e);

/*
 * Write the ELF object to a file.  This will do the following:
 *  - Fill in all the ehdr data.
 *  - Calculate offsets for all the phdrs.
 *  - Call the pre_write functions for all phdrs.
 *  - Write out the phdrs.
 *  - Call all the do_write functions for the phdrs.
 *  - Call all the post_write functions for the phdrs.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_write(struct elfc *e);

/*
 * Read len bytes from the given offset in the elf file and store it in
 * odata.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_read_data(struct elfc *e, GElf_Off off, void *odata, size_t len);

/*
 * Like elfc_read_data(), but allocates the buffer for you.
 */
int elfc_alloc_read_data(struct elfc *e, GElf_Off off,
			 void **odata, size_t len);

/*
 * Convert the given virtual address to a physical address.  The first
 * one is returned.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_vmem_to_pmem(struct elfc *e, GElf_Addr vaddr, GElf_Addr *paddr);

/*
 * Find the given virtual address in the elf file and copy its contents
 * to odata.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_read_vmem(struct elfc *e, GElf_Addr addr, void *odata, size_t len);

/*
 * Like elfc_read_vmem, but uses the physical address.
 */
int elfc_read_pmem(struct elfc *e, GElf_Addr addr, void *odata, size_t len);

/*
 * Like elfc_read_vmem, but allocates the data and returns it.
 */
int elfc_alloc_read_vmem(struct elfc *e, GElf_Addr addr,
			 void **odata, size_t len);

/*
 * Like elfc_read_pmem, but allocates the data and returns it.
 */
int elfc_alloc_read_pmem(struct elfc *e, GElf_Addr addr,
			 void **odata, size_t len);

/*
 * Write to the memory at the given virtual address.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_write_vmem(struct elfc *e, GElf_Addr addr,
		    const void *odata, size_t len);

/*
 * Write to the memory at the given physical address.
 *
 * Returns -1 on error, use elfc_get_errno() to get the errno.
 */
int elfc_write_pmem(struct elfc *e, GElf_Addr addr,
		    const void *odata, size_t len);

/*
 * Returns TRUE if the given address is present in the elf file,
 * false if not.
 */
int elfc_pmem_present(struct elfc *e, GElf_Addr addr, size_t len);
int elfc_vmem_present(struct elfc *e, GElf_Addr addr, size_t len);

/*
 * Get one past the maximum valid physical/virtual address
 */
GElf_Addr elfc_max_paddr(struct elfc *e);
GElf_Addr elfc_max_vaddr(struct elfc *e);

/*
 * Return the Phdr number and offset from the beginning of the Phdr
 * for the given virtual address and length.
 */
int elfc_vmem_offset(struct elfc *e, GElf_Addr addr, size_t len,
		     int *pnum, GElf_Off *off);

/*
 * Like elfc_vmem_offset, but for physical addresses.
 */
int elfc_pmem_offset(struct elfc *e, GElf_Addr addr, size_t len,
		     int *pnum, GElf_Off *off);

/*
 * Like the above two, but returns the file offset.
 */
int elfc_vmem_file_offset(struct elfc *e, GElf_Addr addr, size_t len,
			  GElf_Off *off);
int elfc_pmem_file_offset(struct elfc *e, GElf_Addr addr, size_t len,
			  GElf_Off *off);

/*
 * Return the total size of the file, basically one past the end of
 * the last phdr.
 */
GElf_Off elfc_file_size(struct elfc *e);

/*
 * Lookup the symbol (starting at the given startidx + 1) and search for
 * the given symbol, returning it in "sym".
 * Returns -1 on error, use elfc_get_errno() for the errno.
 */
int elfc_lookup_sym(struct elfc *e, const char *name, GElf_Sym *sym,
		    Elf32_Word startidx, Elf32_Word *symidx);

/*
 * Return the size of a single elf symbols in the file (either
 * sizeof(Elf32_Sym) or sizeof(Elf64_Sym).
 */
Elf32_Word elfc_sym_size_one(struct elfc *e);

/*
 * The number of symbols in the file.
 */
Elf32_Word elfc_num_syms(struct elfc *e);

/*
 * Lookup the symbol at the given index and return it in "sym".
 * Returns -1 on error, use elfc_get_errno() for the errno.
 */
int elfc_get_sym(struct elfc *e, Elf32_Word index, GElf_Sym *sym);

/*
 * Replace the contents of a sym index with the given sym data.
 * Returns -1 on error, use elfc_get_errno() for the errno.
 */
int elfc_set_sym(struct elfc *e, Elf32_Word index, GElf_Sym *sym);

/*
 * Lookup the symbol name at the given index and return it.
 * Same rules as elfc_get_str().
 * Returns NULL on error, use elfc_get_errno() for the errno.
 */
const char *elfc_get_sym_name(struct elfc *e, Elf32_Word index);

/*
 * Get a string from the section header string table given it's index.
 * The return value should not be free or modified and will go away.
 * if the elfc structure is freed.
 *
 * If NULL Is returned, use elfc_get_errno to get the errno.
 */
const char *elfc_get_shstr(struct elfc *e, Elf32_Word index);

/*
 * Get a string from the symbol string table given it's index.  The
 * return value should not be free or modified and will go away.  if
 * the elfc structure is freed.
 *
 * If NULL Is returned, use elfc_get_errno to get the errno.
 */
const char *elfc_get_str(struct elfc *e, Elf32_Word index);

/*
 * Various routines to convert between target order and host order.
 */
GElf_Half elfc_getHalf(struct elfc *e, GElf_Half w);
GElf_Half elfc_putHalf(struct elfc *e, GElf_Half w);
GElf_Word elfc_getWord(struct elfc *e, GElf_Word w);
GElf_Word elfc_putWord(struct elfc *e, GElf_Word w);
GElf_Xword elfc_getXword(struct elfc *e, GElf_Xword w);
GElf_Xword elfc_putXword(struct elfc *e, GElf_Xword w);
GElf_Section elfc_getSection(struct elfc *e, GElf_Section w);
GElf_Section elfc_putSection(struct elfc *e, GElf_Section w);
GElf_Addr elfc_getAddr(struct elfc *e, GElf_Addr w);
GElf_Addr elfc_putAddr(struct elfc *e, GElf_Addr w);
GElf_Off elfc_getOff(struct elfc *e, GElf_Off w);
GElf_Off elfc_putOff(struct elfc *e, GElf_Off w);
unsigned char elfc_getuchar(struct elfc *e, unsigned char w);
unsigned char elfc_putuchar(struct elfc *e, unsigned char w);


#endif /* MY_ELFHND_H */
