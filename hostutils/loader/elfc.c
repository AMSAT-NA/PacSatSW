/*
 * elfc.c
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

#define _FILE_OFFSET_BITS 64

#include <endian.h>
#include <gelf.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

#include "elfc.h"

/*
 * FIXMES:
 *
 * Need to add protection to keep the user from directly modifying symbol
 * and string table data.
 */

#define elfc_align(v, a) (((v) + (a) - 1) & ~((typeof(v)) (a - 1)))

struct elfc_note {
	Elf64_Word type;
	char *name;
	size_t namelen;
	void *data;
	size_t datalen;
};

struct elfc_phdr {
	GElf_Phdr p;
	int idx;
	void *data;
	void *userdata;
	void (*data_free)(struct elfc *e, void *data, void *userdata);
	int (*pre_write)(struct elfc *e, GElf_Phdr *phdr,
			 void *data, void *userdata);
	int (*do_write)(struct elfc *e, int fd, GElf_Phdr *phdr,
			void *data, void *userdata);
	void (*post_write)(struct elfc *e, GElf_Phdr *phdr,
			   void *data, void *userdata);
	int (*get_data)(struct elfc *e, GElf_Phdr *phdr, void *data,
			GElf_Off off,
			void *odata, size_t len, void *userdata);
	int (*set_data)(struct elfc *e, GElf_Phdr *phdr, void *data,
			GElf_Off off,
			const void *idata, size_t len, void *userdata);
};

/* A btree to hold phdrs indexed by physical_address. */
#define BTREE_NODE_SIZE 10
#define btree_val_t struct elfc_phdr *
#define BTREE_NAMES_LOCAL static
#define BEN(s) phdr_phys_ ## s
#define btree_t phdr_phys_btree
#define btree_cmp_key phdr_phys_cmp
#define BTREE_NEEDS BTREE_NEEDS_NEXT

int
phdr_phys_cmp(struct elfc_phdr *val1, struct elfc_phdr *val2)
{
	if (val1->p.p_paddr > val2->p.p_paddr + val2->p.p_filesz - 1)
		return 1;
	if (val2->p.p_paddr > val1->p.p_paddr + val1->p.p_filesz - 1)
		return -1;
	return 0;
}

#include "btree.h"

#undef BEN
#undef btree_t
#undef btree_cmp_key
#undef BTREE_NEEDS
#undef BTREE_NODE_SIZE
#undef btree_val_t
#undef BTREE_NAMES_LOCAL


struct elfc_shdr {
	GElf_Shdr sh;
	void *data;
	void *userdata;
	void (*data_free)(struct elfc *e, void *data, void *userdata);
	int (*pre_write)(struct elfc *e, GElf_Shdr *shdr,
			 void *data, void *userdata);
	int (*do_write)(struct elfc *e, int fd, GElf_Shdr *shdr,
			void *data, void *userdata);
	void (*post_write)(struct elfc *e, GElf_Shdr *shdr,
			   void *data, void *userdata);
	int (*get_data)(struct elfc *e, GElf_Shdr *shdr, void *data,
			GElf_Off off,
			void *odata, size_t len, void *userdata);
	int (*set_data)(struct elfc *e, GElf_Shdr *shdr, void *data,
			GElf_Off off,
			const void *idata, size_t len, void *userdata);
};

struct elfc {
	GElf_Ehdr hdr;
	int eerrno;
	int fd;

	void *userdata;

	GElf_Off after_headers;

	struct elfc_phdr **phdrs;
	Elf32_Word num_phdrs;
	int alloced_phdrs;
	phdr_phys_btree phdr_tree;

	/*
	 * We only hold a dummy section header for putting the
	 * num_phdrs values if it is greater than 65534.
	 */
	struct elfc_shdr *shdrs;
	Elf32_Word num_shdrs;
	int alloced_shdrs;

	/*
	 * Indexes for special sections we use.
	 */
	Elf32_Word shstrtab;
	Elf32_Word strtab;
	Elf32_Word symtab;
	Elf32_Word symtab_size;

	struct elfc_note *notes;
	int num_notes;
	int alloced_notes;
};

static int elfc_read_notes(struct elfc *e);

#define Phdr32_Entries \
	PhdrE(Word,	type);		\
	PhdrE(Off,	offset);	\
	PhdrE(Addr,	vaddr);		\
	PhdrE(Addr,	paddr);		\
	PhdrE(Word,	filesz);	\
	PhdrE(Word,	memsz);		\
	PhdrE(Word,	flags);		\
	PhdrE(Word,	align);

#define Phdr64_Entries \
	PhdrE(Word,	type);		\
	PhdrE(Off,	offset);	\
	PhdrE(Addr,	vaddr);		\
	PhdrE(Addr,	paddr);		\
	PhdrE(Xword,	filesz);	\
	PhdrE(Xword,	memsz);		\
	PhdrE(Word,	flags);		\
	PhdrE(Xword,	align);

static int
extend_phdrs(struct elfc *e)
{
	if (e->num_phdrs == e->alloced_phdrs) {
		struct elfc_phdr **phdrs;

		phdrs = malloc(sizeof(*phdrs) * (e->alloced_phdrs + 32));
		if (!phdrs) {
			e->eerrno = ENOMEM;
			return -1;
		}
		memcpy(phdrs, e->phdrs, sizeof(*phdrs) * e->alloced_phdrs);
		e->alloced_phdrs += 32;
		if (e->phdrs)
			free(e->phdrs);
		e->phdrs = phdrs;
	}
	return 0;
}

int
elfc_insert_phdr(struct elfc *e, int pnum,
		 GElf_Word type, GElf_Addr vaddr, GElf_Addr paddr,
		 GElf_Xword filesz, GElf_Xword memsz, GElf_Word flags,
		 GElf_Word align)
{
	GElf_Off offset = 0;
	int i, rv;
	struct elfc_phdr *p;

	if (pnum > (e->num_phdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}

	p = malloc(sizeof(*p));
	if (!p) {
		e->eerrno = ENOMEM;
		return -1;
	}
	memset(p, 0, sizeof(*p));

	if (extend_phdrs(e) == -1) {
		free(p);
		e->eerrno = ENOMEM;
		return -1;
	}

#define PhdrE(type, name) p->p.p_ ## name = name;
	Phdr64_Entries;
#undef PhdrE

	if (type == PT_LOAD) {
		rv = phdr_phys_add(&e->phdr_tree, p);
		if (rv) {
			free(p);
			e->eerrno = ENOMEM;
			return -1;
		}
	}

	memmove(e->phdrs + pnum + 1, e->phdrs + pnum,
		sizeof(*e->phdrs) * (e->num_phdrs - pnum));

	e->phdrs[pnum] = p;
	e->num_phdrs++;

	for (i = pnum; i < e->num_phdrs; i++)
		e->phdrs[i]->idx = i;

	return pnum;
}

int
elfc_add_phdr(struct elfc *e,
	      GElf_Word type, GElf_Addr vaddr, GElf_Addr paddr,
	      GElf_Xword filesz, GElf_Xword memsz, GElf_Word flags,
	      GElf_Word align)
{
	GElf_Off offset = 0;
	struct elfc_phdr *p;
	int i, rv;

	p = malloc(sizeof(*p));
	if (!p) {
		e->eerrno = ENOMEM;
		return -1;
	}
	memset(p, 0, sizeof(*p));

	if (extend_phdrs(e) == -1) {
		free(p);
		e->eerrno = ENOMEM;
		return -1;
	}

#define PhdrE(type, name) p->p.p_ ## name = name;
	Phdr64_Entries;
#undef PhdrE

	if (type == PT_LOAD) {
		rv = phdr_phys_add(&e->phdr_tree, p);
		if (rv) {
			free(p);
			e->eerrno = ENOMEM;
			return -1;
		}
	}

	i = e->num_phdrs;
	e->phdrs[i] = p;
	p->idx = i;
	e->num_phdrs++;
	return i;
}

int
elfc_del_phdr(struct elfc *e, int pnum)
{
	int i;

	if (pnum >= e->num_phdrs) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (e->phdrs[pnum]->data_free)
		e->phdrs[pnum]->data_free(e, e->phdrs[pnum]->data,
					 e->phdrs[pnum]->userdata);
	free(e->phdrs[pnum]);
	memmove(e->phdrs + pnum, e->phdrs + pnum + 1,
		sizeof(*e->phdrs) * (e->num_phdrs - pnum - 1));
	e->num_phdrs--;

	for (i = pnum; i < e->num_phdrs; i++)
		e->phdrs[i]->idx = i;

	return 0;
}

void
elfc_gen_phdr_free(struct elfc *e, void *data, void *userdata)
{
	if (data)
		free(data);
	if (userdata)
		free(userdata);
}

#define Shdr32_Entries \
	ShdrE(Word,	name);		\
	ShdrE(Word,	type);		\
	ShdrE(Word,	flags);		\
	ShdrE(Addr,	addr);		\
	ShdrE(Off,	offset);	\
	ShdrE(Word,	size);		\
	ShdrE(Word,	link);		\
	ShdrE(Word,	info);		\
	ShdrE(Word,	addralign);	\
	ShdrE(Word,	entsize);

#define Shdr64_Entries \
	ShdrE(Word,	name);		\
	ShdrE(Word,	type);		\
	ShdrE(Xword,	flags);		\
	ShdrE(Addr,	addr);		\
	ShdrE(Off,	offset);	\
	ShdrE(Xword,	size);		\
	ShdrE(Word,	link);		\
	ShdrE(Word,	info);		\
	ShdrE(Xword,	addralign);	\
	ShdrE(Xword,	entsize);

static int
extend_shdrs(struct elfc *e)
{
	if (e->num_shdrs == e->alloced_shdrs) {
		struct elfc_shdr *shdrs;

		shdrs = malloc(sizeof(*shdrs) * (e->alloced_shdrs + 32));
		if (!shdrs) {
			e->eerrno = ENOMEM;
			return -1;
		}
		memcpy(shdrs, e->shdrs, sizeof(*shdrs) * e->alloced_shdrs);
		e->alloced_shdrs += 32;
		if (e->shdrs)
			free(e->shdrs);
		e->shdrs = shdrs;
	}
	return 0;
}

int
elfc_insert_shdr(struct elfc *e, int pnum,
		 GElf_Word name, GElf_Word type, GElf_Xword flags,
		 GElf_Addr addr, GElf_Off offset, GElf_Xword size,
		 GElf_Word link, GElf_Word info, GElf_Xword addralign,
		 GElf_Xword entsize)
{
	if (pnum > (e->num_shdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (extend_shdrs(e) == -1)
		return -1;

	memmove(e->shdrs + pnum + 1, e->shdrs + pnum,
		sizeof(*e->shdrs) * (e->num_shdrs - pnum));

	memset(e->shdrs + pnum, 0, sizeof(*e->shdrs));

#define ShdrE(type, name) e->shdrs[pnum].sh.sh_ ## name = name;
	Shdr64_Entries;
#undef ShdrE
	e->num_shdrs++;
	return pnum;
}

int
elfc_add_shdr(struct elfc *e,
	      GElf_Word name, GElf_Word type, GElf_Xword flags,
	      GElf_Addr addr, GElf_Off offset, GElf_Xword size,
	      GElf_Word link, GElf_Word info, GElf_Xword addralign,
	      GElf_Xword entsize)
{
	int i;

	extend_shdrs(e);

	i = e->num_shdrs;
	memset(&e->shdrs[i], 0, sizeof(e->shdrs[i]));
#define ShdrE(type, name) e->shdrs[i].sh.sh_ ## name = name;
	Shdr64_Entries;
#undef ShdrE
	e->num_shdrs++;
	return i;
}

int
elfc_del_shdr(struct elfc *e, int pnum)
{
	if (pnum >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (e->shdrs[pnum].data_free)
		e->shdrs[pnum].data_free(e, e->shdrs[pnum].data,
					 e->shdrs[pnum].userdata);
	memmove(e->shdrs + pnum, e->shdrs + pnum + 1,
		sizeof(*e->shdrs) * (e->num_shdrs - pnum - 1));
	e->num_shdrs--;
	return 0;
}

void
elfc_gen_shdr_free(struct elfc *e, void *data, void *userdata)
{
	if (data)
		free(data);
	if (userdata)
		free(userdata);
}

int
elfc_tmpfd(void)
{
	char *tmpdir;
	static char *rname = "elfcXXXXXX";
	char *fname;
	int fd;

	tmpdir = getenv("TMPDIR");
	if (!tmpdir)
		tmpdir = "/tmp";

	fname = malloc(strlen(tmpdir) + strlen(rname) + 12);
	if (!fname) {
		errno = ENOMEM;
		return -1;
	}
	sprintf(fname, "%s/%s.%d", tmpdir, rname, (int) getpid());
	fd = open(fname, O_RDWR | O_TRUNC | O_CREAT, S_IRUSR | S_IWUSR);
	if (fd == -1) {
		free(fname);
		return -1;
	}
	unlink(fname);
	free(fname);
	return fd;
}

int
elfc_copy_fd_range(int out, int in, size_t size)
{
	char *buf;
	size_t buf_size = 1024 * 1024;
	int rv = 0;

	if (buf_size > size)
		buf_size = size;
	buf = malloc(buf_size);
	if (!buf) {
		errno = ENOMEM;
		return -1;
	}
	while (size) {
		int iosize = buf_size;
		if (iosize > size)
			iosize = size;
		rv = read(in, buf, iosize);
		if (rv == -1)
			goto out;
		if (rv != iosize) {
			rv = -1;
			errno = ERANGE;
			goto out;
		}
		rv = write(out, buf, iosize);
		if (rv == -1)
			goto out;
		if (rv != iosize) {
			rv = -1;
			errno = ERANGE;
			goto out;
		}
		size -= iosize;
	}
out:
	free(buf);
	return rv;
}

struct elfc_tmpfile {
	int fd;
	unsigned int fd_open_refcount;
	unsigned int refcount;
};

struct elfc_tmpfile_data {
	struct elfc_tmpfile *tf;

	/* File offset of the data. */
	off_t offset;
	size_t len;
	bool opened;
};

static void *
elfc_tmpfile_alloc(struct elfc *e, void **allocdata)
{
	struct elfc_tmpfile *tf = *allocdata;
	struct elfc_tmpfile_data *td;

	td = malloc(sizeof(*td));
	if (!td)
		return NULL;

	if (!tf) {
		tf = malloc(sizeof(*tf));
		if (!tf) {
			free(td);
			return NULL;
		}
		tf->fd_open_refcount = 0;
		tf->refcount = 0;
		tf->fd = -1;
		*allocdata = tf;
	}

	tf->refcount++;
	td->tf = tf;
	td->offset = 0;
	td->len = 0;
	td->opened = false;

	return td;
}

/*
 * Create a copy of the contents in a temparary file.  This way if we
 * are reading and writing the same file, the data won't be clobbered.
 */
static int
elfc_phdr_tmpfile_pre_write(struct elfc *e, GElf_Phdr *phdr,
			    void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;
	int fd;
	int rv;

	if (tf->fd == -1) {
		tf->fd = elfc_tmpfd();
		if (tf->fd == -1) {
			e->eerrno = errno;
			return -1;
		}
	}
	tf->fd_open_refcount++;
	td->opened = true;

	fd = elfc_get_fd(e);
	rv = lseek(fd, phdr->p_offset, SEEK_SET);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}

	/* Save the current offset in the file so we can find it again. */
	td->offset = lseek(tf->fd, 0, SEEK_CUR);
	if (td->offset == -1) {
		e->eerrno = errno;
		return -1;
	}
	td->len = phdr->p_filesz;

	rv = elfc_copy_fd_range(tf->fd, fd, phdr->p_filesz);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	return 0;
}

static int
elfc_phdr_tmpfile_do_write(struct elfc *e, int fd, GElf_Phdr *phdr,
			   void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;
	int rv;

	rv = lseek(tf->fd, td->offset, SEEK_SET);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}

	rv = elfc_copy_fd_range(fd, tf->fd, phdr->p_filesz);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}

	return 0;
}

static void
elfc_phdr_tmpfile_post_write(struct elfc *e, GElf_Phdr *phdr,
			     void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;

	tf->fd_open_refcount--;
	td->opened = false;
	if (tf->fd_open_refcount == 0) {
		close(tf->fd);
		tf->fd = -1;
	}
}

static int
elfc_phdr_tmpfile_get_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			   GElf_Off off, void *odata, size_t len,
			   void *userdata)
{
	int rv;

	if ((off > phdr->p_filesz) || ((off + len) > phdr->p_filesz)) {
		errno = EINVAL;
		return -1;
	}
	rv = lseek(e->fd, off + phdr->p_offset, SEEK_SET);
	if (rv == -1)
		return -1;
	rv = read(e->fd, odata, len);
	if (rv == -1)
		return -1;
	if (rv != len) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

static int
elfc_phdr_tmpfile_set_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			   GElf_Off off, const void *idata, size_t len,
			   void *userdata)
{
	int rv;

	if ((off > phdr->p_filesz) || ((off + len) > phdr->p_filesz)) {
		errno = EINVAL;
		return -1;
	}
	rv = lseek(e->fd, off + phdr->p_offset, SEEK_SET);
	if (rv == -1)
		return -1;
	rv = write(e->fd, idata, len);
	if (rv == -1)
		return -1;
	if (rv != len) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

static void
elfc_phdr_tmpfile_free(struct elfc *e, void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;

	if (!td)
		return;
	tf = td->tf;
	if (tf) {
		if (td->opened)
			elfc_phdr_tmpfile_post_write(e, NULL, NULL, userdata);
		tf->refcount--;
		if (tf->refcount == 0)
			free(tf);
	}
	free(td);
}

int
elfc_phdr_block_get_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			 GElf_Off off, void *odata, size_t len,
			 void *userdata)
{
	if ((off > phdr->p_filesz) || ((off + len) > phdr->p_filesz)) {
		errno = EINVAL;
		return -1;
	}

	memcpy(odata, ((char *) data) + off, len);
	return 0;
}

int
elfc_phdr_block_set_data(struct elfc *e, GElf_Phdr *phdr, void *data,
			 GElf_Off off, const void *idata, size_t len,
			 void *userdata)
{
	if ((off > phdr->p_filesz) || ((off + len) > phdr->p_filesz)) {
		errno = EINVAL;
		return -1;
	}

	memcpy(data, ((char *) idata) + off, len);
	return 0;
}

int
elfc_phdr_block_do_write(struct elfc *e, int fd, GElf_Phdr *phdr,
			 void *data, void *userdata)
{
	int rv;

	rv = write(fd, data, phdr->p_filesz);
	if (rv == -1)
		return -1;
	if (rv != phdr->p_filesz) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

int
elfc_set_phdr_data(struct elfc *e, int pnum, void *data,
		   void (*free_func)(struct elfc *e, void *data,
				     void *userdata),
		   int (*pre_write)(struct elfc *e, GElf_Phdr *phdr,
				    void *data, void *userdata),
		   int (*do_write)(struct elfc *e, int fd, GElf_Phdr *phdr,
				   void *data, void *userdata),
		   void (*post_write)(struct elfc *e, GElf_Phdr *phdr,
				      void *data, void *userdata),
		   int (*get_data)(struct elfc *e, GElf_Phdr *phdr, void *data,
				   GElf_Off off, void *odata, size_t len,
				   void *userdata),
		   int (*set_data)(struct elfc *e, GElf_Phdr *phdr, void *data,
				   GElf_Off off, const void *idata, size_t len,
				   void *userdata),
		   void *userdata)
{
	if (pnum >= e->num_phdrs) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (e->phdrs[pnum]->data_free)
		e->phdrs[pnum]->data_free(e, e->phdrs[pnum]->data,
					  e->phdrs[pnum]->userdata);
	e->phdrs[pnum]->data = data;
	e->phdrs[pnum]->data_free = free_func;
	e->phdrs[pnum]->pre_write = pre_write;
	e->phdrs[pnum]->do_write = do_write;
	e->phdrs[pnum]->post_write = post_write;
	e->phdrs[pnum]->get_data = get_data;
	e->phdrs[pnum]->set_data = set_data;
	e->phdrs[pnum]->userdata = userdata;
	return 0;
}

int
elfc_get_num_phdrs(struct elfc *e)
{
	return e->num_phdrs;
}

int
elfc_get_phdr_offset(struct elfc *e, int pnum, GElf_Off *off)
{
	if (pnum >= e->num_phdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	*off = e->phdrs[pnum]->p.p_offset;
	return 0;
}

int
elfc_get_phdr(struct elfc *e, int pnum, GElf_Phdr *hdr)
{
	if (pnum >= e->num_phdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	*hdr = e->phdrs[pnum]->p;
	return 0;
}

int
elfc_set_phdr_offset(struct elfc *e, int pnum, GElf_Off offset)
{
	if (pnum >= e->num_phdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	e->phdrs[pnum]->p.p_offset = offset;
	return 0;
}

int
elfc_phdr_read(struct elfc *e, int pnum, GElf_Off off,
	       void *odata, size_t len)
{
	int rv;

	if (pnum > (e->num_phdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->phdrs[pnum]->get_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	rv = e->phdrs[pnum]->get_data(e, &e->phdrs[pnum]->p,
				      e->phdrs[pnum]->data,
				      off, odata, len,
				      e->phdrs[pnum]->userdata);
	if (rv)
		e->eerrno = errno;
	return rv;
}

int
elfc_phdr_write(struct elfc *e, int pnum, GElf_Off off,
		const void *odata, size_t len)
{
	int rv;

	if (pnum > (e->num_phdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->phdrs[pnum]->set_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	rv = e->phdrs[pnum]->set_data(e, &e->phdrs[pnum]->p,
				      e->phdrs[pnum]->data,
				      off, odata, len,
				      e->phdrs[pnum]->userdata);
	if (rv)
		e->eerrno = errno;
	return rv;
}

int
elfc_phdr_alloc_read(struct elfc *e, int pnum, GElf_Off off,
		     void **odata, size_t len)
{
	int rv;
	char *buf;

	if (pnum > (e->num_phdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->phdrs[pnum]->get_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	buf = malloc(len);
	if (!buf) {
		e->eerrno = ENOMEM;
		return -1;
	}
	rv = e->phdrs[pnum]->get_data(e, &e->phdrs[pnum]->p,
				      e->phdrs[pnum]->data,
				      off, buf, len, e->phdrs[pnum]->userdata);
	if (rv) {
		free(buf);
		e->eerrno = errno;
	} else
		*odata = buf;

	return rv;
}

/*
 * Create a copy of the contents in a temparary file.  This way if we
 * are reading and writing the same file, the data won't be clobbered.
 */
static int
elfc_shdr_tmpfile_pre_write(struct elfc *e, GElf_Shdr *shdr,
			    void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;
	int fd;
	int rv;

	if (tf->fd == -1) {
		tf->fd = elfc_tmpfd();
		if (tf->fd == -1) {
			e->eerrno = errno;
			return -1;
		}
	}
	tf->fd_open_refcount++;
	td->opened = true;

	fd = elfc_get_fd(e);
	rv = lseek(fd, shdr->sh_offset, SEEK_SET);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}

	/* Save the current offset in the file so we can find it again. */
	td->offset = lseek(tf->fd, 0, SEEK_CUR);
	if (td->offset == -1) {
		e->eerrno = errno;
		return -1;
	}
	td->len = shdr->sh_size;

	rv = elfc_copy_fd_range(tf->fd, fd, shdr->sh_size);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	return 0;
}

static int
elfc_shdr_tmpfile_do_write(struct elfc *e, int fd, GElf_Shdr *shdr,
			   void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;
	int rv;

	rv = lseek(tf->fd, td->offset, SEEK_SET);
	if (rv == -1)
		return -1;

	rv = elfc_copy_fd_range(fd, tf->fd, shdr->sh_size);
	if (rv == -1)
		return -1;

	close(tf->fd);
	tf->fd = -1;
	return 0;
}

static void
elfc_shdr_tmpfile_post_write(struct elfc *e, GElf_Shdr *shdr,
			     void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;

	tf->fd_open_refcount--;
	td->opened = false;
	if (tf->fd_open_refcount == 0) {
		close(tf->fd);
		tf->fd = -1;
	}
}

static int
elfc_shdr_tmpfile_get_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			   GElf_Off off, void *odata, size_t len,
			   void *userdata)
{
	int rv;

	if ((off > shdr->sh_size) || ((off + len) > shdr->sh_size)) {
		errno = EINVAL;
		return -1;
	}
	rv = lseek(e->fd, off + shdr->sh_offset, SEEK_SET);
	if (rv == -1)
		return -1;
	rv = read(e->fd, odata, len);
	if (rv == -1)
		return -1;
	if (rv != len) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

static int
elfc_shdr_tmpfile_set_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			   GElf_Off off, const void *idata, size_t len,
			   void *userdata)
{
	int rv;

	if ((off > shdr->sh_size) || ((off + len) > shdr->sh_size)) {
		errno = EINVAL;
		return -1;
	}
	rv = lseek(e->fd, off + shdr->sh_offset, SEEK_SET);
	if (rv == -1)
		return -1;
	rv = write(e->fd, idata, len);
	if (rv == -1)
		return -1;
	if (rv != len) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

static void
elfc_shdr_tmpfile_free(struct elfc *e, void *data, void *userdata)
{
	struct elfc_tmpfile_data *td = userdata;
	struct elfc_tmpfile *tf = td->tf;

	/* data is only set here in the case of symbol and string tables. */
	if (data)
		free(data);

	if (!td)
		return;
	tf = td->tf;
	if (tf) {
		if (td->opened)
			elfc_shdr_tmpfile_post_write(e, NULL, NULL, userdata);
		tf->refcount--;
		if (tf->refcount == 0)
			free(tf);
	}
	free(td);
}

int
elfc_shdr_block_get_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			 GElf_Off off, void *odata, size_t len,
			 void *userdata)
{
	if ((off > shdr->sh_size) || ((off + len) > shdr->sh_size)) {
		errno = EINVAL;
		return -1;
	}

	memcpy(odata, ((char *) data) + off, len);
	return 0;
}

int
elfc_shdr_block_set_data(struct elfc *e, GElf_Shdr *shdr, void *data,
			 GElf_Off off, const void *idata, size_t len,
			 void *userdata)
{
	if ((off > shdr->sh_size) || ((off + len) > shdr->sh_size)) {
		errno = EINVAL;
		return -1;
	}

	memcpy(data, ((char *) idata) + off, len);
	return 0;
}

int
elfc_shdr_block_do_write(struct elfc *e, int fd, GElf_Shdr *shdr,
			 void *data, void *userdata)
{
	int rv;

	rv = write(fd, data, shdr->sh_size);
	if (rv == -1)
		return -1;
	if (rv != shdr->sh_size) {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

int
elfc_set_shdr_data(struct elfc *e, int snum, void *data,
		   void (*free_func)(struct elfc *e, void *data,
				     void *userdata),
		   int (*pre_write)(struct elfc *e, GElf_Shdr *shdr,
				    void *data, void *userdata),
		   int (*do_write)(struct elfc *e, int fd, GElf_Shdr *shdr,
				   void *data, void *userdata),
		   void (*post_write)(struct elfc *e, GElf_Shdr *shdr,
				      void *data, void *userdata),
		   int (*get_data)(struct elfc *e, GElf_Shdr *shdr, void *data,
				   GElf_Off off, void *odata, size_t len,
				   void *userdata),
		   int (*set_data)(struct elfc *e, GElf_Shdr *shdr, void *data,
				   GElf_Off off, const void *idata, size_t len,
				   void *userdata),
		   void *userdata)
{
	if (snum >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (e->shdrs[snum].data_free)
		e->shdrs[snum].data_free(e, e->shdrs[snum].data,
					 e->shdrs[snum].userdata);
	e->shdrs[snum].data = data;
	e->shdrs[snum].data_free = free_func;
	e->shdrs[snum].pre_write = pre_write;
	e->shdrs[snum].do_write = do_write;
	e->shdrs[snum].post_write = post_write;
	e->shdrs[snum].get_data = get_data;
	e->shdrs[snum].set_data = set_data;
	e->shdrs[snum].userdata = userdata;
	return 0;
}

int
elfc_get_num_shdrs(struct elfc *e)
{
	return e->num_shdrs;
}

int
elfc_get_shdr_offset(struct elfc *e, int snum, GElf_Off *off)
{
	if (snum >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	*off = e->shdrs[snum].sh.sh_offset;
	return 0;
}

int
elfc_get_shdr(struct elfc *e, int snum, GElf_Shdr *hdr)
{
	if (snum >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	*hdr = e->shdrs[snum].sh;
	return 0;
}

int
elfc_set_shdr_offset(struct elfc *e, int snum, GElf_Off offset)
{
	if (snum >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}
	e->shdrs[snum].sh.sh_offset = offset;
	return 0;
}

int
elfc_shdr_read(struct elfc *e, int snum, GElf_Off off,
	       void *odata, size_t len)
{
	int rv;

	if (snum > (e->num_shdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->shdrs[snum].get_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	rv = e->shdrs[snum].get_data(e, &e->shdrs[snum].sh, e->shdrs[snum].data,
				     off, odata, len, e->shdrs[snum].userdata);
	if (rv)
		e->eerrno = errno;
	return rv;
}

int
elfc_shdr_write(struct elfc *e, int snum, GElf_Off off,
		const void *odata, size_t len)
{
	int rv;

	if (snum > (e->num_shdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->shdrs[snum].set_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	rv = e->shdrs[snum].set_data(e, &e->shdrs[snum].sh, e->shdrs[snum].data,
				     off, odata, len, e->shdrs[snum].userdata);
	if (rv)
		e->eerrno = errno;
	return rv;
}

int
elfc_shdr_alloc_read(struct elfc *e, int snum, GElf_Off off,
		     void **odata, size_t len)
{
	int rv;
	char *buf;

	if (snum > (e->num_shdrs + 1)) {
		e->eerrno = EINVAL;
		return -1;
	}
	if (!e->shdrs[snum].get_data) {
		e->eerrno = ENOSYS;
		return -1;
	}
	buf = malloc(len);
	if (!buf) {
		e->eerrno = ENOMEM;
		return -1;
	}
	rv = e->shdrs[snum].get_data(e, &e->shdrs[snum].sh, e->shdrs[snum].data,
				     off, buf, len, e->shdrs[snum].userdata);
	if (rv) {
		free(buf);
		e->eerrno = errno;
	} else
		*odata = buf;

	return rv;
}

static int
elfc_add_note_nocheck(struct elfc *e, Elf32_Word type,
		      const char *name, int namelen,
		      const void *data, int datalen)
{
	if (e->num_notes == e->alloced_notes) {
		struct elfc_note *notes;

		notes = malloc(sizeof(*notes) * (e->alloced_notes + 32));
		if (!notes) {
			e->eerrno = ENOMEM;
			return -1;
		}
		memcpy(notes, e->notes, sizeof(*notes) * e->alloced_notes);
		e->alloced_notes += 32;
		if (e->notes)
			free(e->notes);
		e->notes = notes;
	}

	e->notes[e->num_notes].type = type;
	e->notes[e->num_notes].name = malloc(namelen + 1);
	if (!e->notes[e->num_notes].name) {
		e->eerrno = ENOMEM;
		return -1;
	}
	e->notes[e->num_notes].data = malloc(datalen);
	if (!e->notes[e->num_notes].data) {
		free(e->notes[e->num_notes].name);
		e->eerrno = ENOMEM;
		return -1;
	}
	memcpy(e->notes[e->num_notes].name, name, namelen);
	e->notes[e->num_notes].name[namelen] = '\0';
	e->notes[e->num_notes].namelen = namelen;
	memcpy(e->notes[e->num_notes].data, data, datalen);
	e->notes[e->num_notes].datalen = datalen;
	e->num_notes++;
	return 0;
}

int
elfc_add_note(struct elfc *e, Elf32_Word type,
	      const char *name, int namelen,
	      const void *data, int datalen)
{
	if (!e->notes && (e->fd != -1)) {
		int rv = elfc_read_notes(e);
		if (rv == -1)
			return rv;
	}

	return elfc_add_note_nocheck(e, type, name, namelen, data, datalen);
}

#define elfc_accessor(name, type)	\
void						\
elfc_set ## name(struct elfc *e, type name)	\
{						\
	e->hdr.e_ ## name = name;		\
}						\
type						\
elfc_get ## name(struct elfc *e)		\
{						\
	return e->hdr.e_ ## name;		\
}

elfc_accessor(machine, GElf_Half);
elfc_accessor(type, GElf_Half);
elfc_accessor(entry, GElf_Addr);

void
elfc_setclass(struct elfc *e, unsigned char class)
{
	e->hdr.e_ident[EI_CLASS] = class;
}

unsigned char
elfc_getclass(struct elfc *e)
{
	return e->hdr.e_ident[EI_CLASS];
}

void
elfc_setencoding(struct elfc *e, unsigned char encoding)
{
	e->hdr.e_ident[EI_DATA] = encoding;
}

unsigned char
elfc_getencoding(struct elfc *e)
{
	return e->hdr.e_ident[EI_DATA];
}

static int elfarch =
#ifdef __x86_64__
	EM_X86_64
#elif defined(__mips__)
	EM_MIPS
#else
	EM_NONE
#endif
	;

static int elfendian =
#if (__BYTE_ORDER == __LITTLE_ENDIAN)
	ELFDATA2LSB
#elif (__BYTE_ORDER == __BIG_ENDIAN)
	ELFDATA2MSB
#else
	ELFDATANONE
#endif
	;

static int elfclass = ELFCLASSNONE;

#define elfc_getput(type, len)				\
GElf_ ## type						\
elfc_get ## type(struct elfc *e, GElf_## type w)	\
{							\
	if (e->hdr.e_ident[EI_DATA] == ELFDATA2LSB)	\
		return le ## len ## toh(w);		\
	else						\
		return be ## len ## toh(w);		\
}							\
GElf_ ## type						\
elfc_put ## type(struct elfc *e, GElf_## type w)	\
{							\
	if (e->hdr.e_ident[EI_DATA] == ELFDATA2LSB)	\
		return htole ## len(w);			\
	else						\
		return htobe ## len(w);			\
}

elfc_getput(Half, 16)
elfc_getput(Word, 32)
elfc_getput(Xword, 64)
elfc_getput(Section, 16)

GElf_Addr
elfc_getAddr(struct elfc *e, GElf_Addr w)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return elfc_getWord(e, w);
	else
		return elfc_getXword(e, w);
}

GElf_Off
elfc_getOff(struct elfc *e, GElf_Off w)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return elfc_getWord(e, w);
	else
		return elfc_getXword(e, w);
}

GElf_Addr
elfc_putAddr(struct elfc *e, GElf_Addr w)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return elfc_putWord(e, w);
	else
		return elfc_putXword(e, w);
}

GElf_Off
elfc_putOff(struct elfc *e, GElf_Off w)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return elfc_putWord(e, w);
	else
		return elfc_putXword(e, w);
}

unsigned char
elfc_getuchar(struct elfc *e, unsigned char v)
{
	return v;
}

unsigned char
elfc_putuchar(struct elfc *e, unsigned char v)
{
	return v;
}

int
elfc_read_data(struct elfc *e, GElf_Off off, void *odata, size_t len)
{
	int rv;

	rv = lseek(e->fd, off, SEEK_SET);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	rv = read(e->fd, odata, len);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != len) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

int
elfc_alloc_read_data(struct elfc *e, GElf_Off off, void **odata, size_t len)
{
	void *buf = malloc(len);
	int rv;

	if (!buf) {
		e->eerrno = ENOMEM;
		return -1;
	}
	rv = elfc_read_data(e, off, buf, len);
	if (rv == -1)
		free(buf);
	else
		*odata = buf;

	return rv;
}

GElf_Off
elfc_file_size(struct elfc *e)
{
	int i;
	GElf_Off s_end;
	GElf_Off rv = 0;

	for (i = 0; i < e->num_phdrs; i++) {
		s_end = e->phdrs[i]->p.p_offset + e->phdrs[i]->p.p_filesz;
		if (s_end > rv)
			rv = s_end;
	}
	return rv;
}

int
elfc_vmem_offset(struct elfc *e, GElf_Addr addr, size_t len,
		 int *pnum, GElf_Off *off)
{
	int i;
	GElf_Addr s_beg;
	GElf_Addr s_end;

	for (i = 0; i < e->num_phdrs; i++) {
		s_beg = e->phdrs[i]->p.p_vaddr;
		s_end = s_beg + e->phdrs[i]->p.p_filesz;

		if ((addr >= s_beg) && ((addr + len) < s_end)) {
			*off = addr - s_beg;
			*pnum = i;
			return 0;
		}
	}
	e->eerrno = ENOENT;
	return -1;
}

int
elfc_pmem_offset(struct elfc *e, GElf_Addr addr, size_t len,
		 int *pnum, GElf_Off *off)
{
	struct elfc_phdr p, *v;
	int rv;
	GElf_Addr s_beg, s_end;

	/* Find an overlap with the data. */
	p.p.p_paddr = addr;
	p.p.p_filesz = len;
	rv = phdr_phys_search(&e->phdr_tree, &p, &v, BTREE_NO_CLOSEST);
	while (!rv) {
	    s_beg = v->p.p_paddr;
	    s_end= s_beg + v->p.p_filesz;

	    /* Make sure the phdr contains the entire range. */
	    if (addr < s_beg || (addr + len) > s_end) {
		/*
		 * Duplicates are allowed, so go through the tree until
		 * we find one that doesn't overlap.
		 */
		rv = phdr_phys_next(&e->phdr_tree, v, &v);
		if (rv || phdr_phys_cmp(&p, v) != 0)
		    break;
	    } else {
		break;
	    }
	}
	if (rv) {
		e->eerrno = ENOENT;
		return -1;
	}

	*pnum = v->idx;
	*off = addr - s_beg;

	return 0;
}

int
elfc_pmem_present(struct elfc *e, GElf_Addr addr, size_t len)
{
	GElf_Off off;
	int pnum;

	if (elfc_pmem_offset(e, addr, len, &pnum, &off) == -1)
		return 0;
	return 1;
}

int
elfc_vmem_present(struct elfc *e, GElf_Addr addr, size_t len)
{
	GElf_Off off;
	int pnum;

	if (elfc_vmem_offset(e, addr, len, &pnum, &off) == -1)
		return 0;
	return 1;
}

GElf_Addr
elfc_max_paddr(struct elfc *e)
{
	int i;
	GElf_Addr max = 0;
	GElf_Addr s_end;

	for (i = 0; i < e->num_phdrs; i++) {
		s_end = e->phdrs[i]->p.p_paddr + e->phdrs[i]->p.p_filesz;
		if (max < s_end)
			max = s_end;
	}
	return max;
}

GElf_Addr
elfc_max_vaddr(struct elfc *e)
{
	int i;
	GElf_Addr max = 0;
	GElf_Addr s_end;

	for (i = 0; i < e->num_phdrs; i++) {
		s_end = e->phdrs[i]->p.p_vaddr + e->phdrs[i]->p.p_filesz;
		if (max < s_end)
			max = s_end;
	}
	return max;
}

int
elfc_vmem_file_offset(struct elfc *e, GElf_Addr addr, size_t len,
		      GElf_Off *off)
{
	int rv;
	GElf_Off poff;
	int pnum;

	rv = elfc_vmem_offset(e, addr, len, &pnum, &poff);
	if (rv == -1)
		return -1;
	*off = poff + e->phdrs[pnum]->p.p_offset;
	return 0;
}

int
elfc_pmem_file_offset(struct elfc *e, GElf_Addr addr, size_t len,
		      GElf_Off *off)
{
	int rv;
	GElf_Off poff;
	int pnum;

	rv = elfc_pmem_offset(e, addr, len, &pnum, &poff);
	if (rv == -1)
		return -1;
	*off = poff + e->phdrs[pnum]->p.p_offset;
	return 0;
}

int elfc_vmem_to_pmem(struct elfc *e, GElf_Addr vaddr, GElf_Addr *paddr)
{
	int i;
	GElf_Addr s_beg;
	GElf_Addr s_end;

	for (i = 0; i < e->num_phdrs; i++) {
		s_beg = e->phdrs[i]->p.p_vaddr;
		s_end = s_beg + e->phdrs[i]->p.p_filesz;

		if ((vaddr >= s_beg) && (vaddr < s_end)) {
			*paddr = e->phdrs[i]->p.p_paddr + (vaddr - s_beg);
			return 0;
		}
	}
	e->eerrno = ENOENT;
	return -1;
}

int
elfc_read_vmem(struct elfc *e, GElf_Addr addr, void *odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_vmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_read(e, pnum, off, odata, len);
}

int
elfc_read_pmem(struct elfc *e, GElf_Addr addr, void *odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_pmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_read(e, pnum, off, odata, len);
}

int
elfc_alloc_read_vmem(struct elfc *e, GElf_Addr addr, void **odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_vmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_alloc_read(e, pnum, off, odata, len);
}

int
elfc_alloc_read_pmem(struct elfc *e, GElf_Addr addr, void **odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_pmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_alloc_read(e, pnum, off, odata, len);
}

int
elfc_write_vmem(struct elfc *e, GElf_Addr addr, const void *odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_vmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_write(e, pnum, off, odata, len);
}

int
elfc_write_pmem(struct elfc *e, GElf_Addr addr, const void *odata, size_t len)
{
	int rv;
	GElf_Off off;
	int pnum;

	rv = elfc_pmem_offset(e, addr, len, &pnum, &off);
	if (rv == -1)
		return -1;
	return elfc_phdr_write(e, pnum, off, odata, len);
}


#define Ehdr_Entries \
	EhdrE(Half,	type);		\
	EhdrE(Half,	machine);	\
	EhdrE(Word,	version);	\
	EhdrE(Addr,	entry);		\
	EhdrE(Off,	phoff);		\
	EhdrE(Off,	shoff);		\
	EhdrE(Word,	flags);		\
	EhdrE(Half,	ehsize);	\
	EhdrE(Half,	phentsize);	\
	EhdrE(Half,	phnum);		\
	EhdrE(Half,	shentsize);	\
	EhdrE(Half,	shnum);		\
	EhdrE(Half,	shstrndx)

static int
read_elf32_ehdr(struct elfc *e)
{
	Elf32_Ehdr e32;
	int rv;

	/* Re-read e_ident. */
	if (lseek(e->fd, 0, SEEK_SET) == -1) {
		e->eerrno = errno;
		return -1;
	}
	rv = read(e->fd, &e32, sizeof(e32));
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != sizeof(e32)) {
		e->eerrno = EINVAL;
		return -1;
	}
#define EhdrE(type, name) e->hdr.e_ ## name = elfc_get ## type(e, e32.e_ ## name);
	Ehdr_Entries;
#undef EhdrE
	return 0;
}

static int
read_elf64_ehdr(struct elfc *e)
{
	Elf64_Ehdr e64;
	int rv;

	/* Re-read e_ident. */
	if (lseek(e->fd, 0, SEEK_SET) == -1) {
		e->eerrno = errno;
		return -1;
	}
	rv = read(e->fd, &e64, sizeof(e64));
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != sizeof(e64)) {
		e->eerrno = EINVAL;
		return -1;
	}
#define EhdrE(type, name) e->hdr.e_ ## name = elfc_get ## type(e, e64.e_ ## name);
	Ehdr_Entries;
#undef EhdrE
	return 0;
}

static int
write_elf32_ehdr(struct elfc *e)
{
	Elf32_Ehdr e32;
	int rv;

	memcpy(e32.e_ident, e->hdr.e_ident, sizeof(e32.e_ident));
#define EhdrE(type, name) e32.e_ ## name = elfc_put ## type(e, e->hdr.e_ ## name);
	Ehdr_Entries;
#undef EhdrE

	rv = write(e->fd, &e32, sizeof(e32));
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != sizeof(e32)) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static int
write_elf64_ehdr(struct elfc *e)
{
	Elf64_Ehdr e64;
	int rv;

	memcpy(e64.e_ident, e->hdr.e_ident, sizeof(e64.e_ident));
#define EhdrE(type, name) e64.e_ ## name = elfc_put ## type(e, e->hdr.e_ ## name);
	Ehdr_Entries;
#undef EhdrE

	rv = write(e->fd, &e64, sizeof(e64));
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != sizeof(e64)) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static void
free_notes(struct elfc *e)
{
	int i;

	if (!e->notes)
		return;
	for (i = 0; i < e->num_notes; i++) {
		free(e->notes[i].name);
		free(e->notes[i].data);
	}
	free(e->notes);
	e->notes = NULL;
	e->num_notes = 0;
	e->alloced_notes = 0;
}

static int
get_elf32_note(struct elfc *e, char *buf, size_t len)
{
	Elf32_Nhdr *nhdr = (Elf32_Nhdr *) buf;
	GElf_Word namesz, descsz, type;
	char *nameptr, *descptr;
	int rv;

	if (len < sizeof(*nhdr))
		return 0;

	namesz = elfc_getWord(e, nhdr->n_namesz);
	descsz = elfc_getWord(e, nhdr->n_descsz);
	type = elfc_getWord(e, nhdr->n_type);

	if (len < sizeof(*nhdr) + namesz + descsz) {
		e->eerrno = EINVAL;
		return -1;
	}

	nameptr = buf + sizeof(*nhdr);
	descptr = nameptr + elfc_align(namesz, sizeof(GElf_Word));
	rv = elfc_add_note_nocheck(e, type, nameptr, namesz, descptr, descsz);
	if (rv == -1)
		return -1;
	descptr = descptr + elfc_align(descsz, sizeof(GElf_Word));

	return descptr - buf;
}

static int
get_elf64_note(struct elfc *e, char *buf, size_t len)
{
	Elf64_Nhdr *nhdr = (Elf64_Nhdr *) buf;
	GElf_Word namesz, descsz, type;
	char *nameptr, *descptr;
	int rv;

	if (len < sizeof(*nhdr))
		return 0;

	namesz = elfc_getWord(e, nhdr->n_namesz);
	descsz = elfc_getWord(e, nhdr->n_descsz);
	type = elfc_getWord(e, nhdr->n_type);

	if (len < sizeof(*nhdr) + namesz + descsz) {
		e->eerrno = EINVAL;
		return -1;
	}

	nameptr = buf + sizeof(*nhdr);
	descptr = nameptr + elfc_align(namesz, sizeof(GElf_Word));
	rv = elfc_add_note_nocheck(e, type, nameptr, namesz, descptr, descsz);
	if (rv == -1)
		return -1;
	descptr = descptr + elfc_align(descsz, sizeof(GElf_Word));

	return descptr - buf;
}

static int
elfc_read_notes(struct elfc *e)
{
	int i, rv;

	free_notes(e);

	for (i = 0; i < e->num_phdrs; i++) {
		void *buf;
		char *nbuf;
		size_t size;

		if (e->phdrs[i]->p.p_type != PT_NOTE)
			continue;

		size = e->phdrs[i]->p.p_filesz;
		rv = elfc_alloc_read_data(e, e->phdrs[i]->p.p_offset,
					  &buf, size);
		if (rv == -1)
			return -1;

		nbuf = buf;
		for (;;) {
			if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
				rv = get_elf32_note(e, nbuf, size);
			else
				rv = get_elf64_note(e, nbuf, size);
			if (rv == -1)
				return -1;
			if (rv == 0)
				break;
			nbuf += rv;
			size -= rv;
		}

		free(buf);

		/* Once we load a note phdr, we delete it. */
		elfc_del_phdr(e, i);
		i--;
	}

	return 0;
}

static int
put_elf32_note(struct elfc *e, int nnum, char *buf, size_t len)
{
	Elf32_Nhdr *nhdr = (Elf32_Nhdr *) buf;
	GElf_Off size, aligned;

	if (nnum > e->num_notes) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (len < sizeof(*nhdr)
	    + e->notes[nnum].namelen + e->notes[nnum].datalen) {
		e->eerrno = EINVAL;
		return -1;
	}

	nhdr->n_namesz = elfc_putWord(e, e->notes[nnum].namelen);
	nhdr->n_descsz = elfc_getWord(e, e->notes[nnum].datalen);
	nhdr->n_type = elfc_getWord(e, e->notes[nnum].type);
	size = sizeof(*nhdr);
	memcpy(buf + size, e->notes[nnum].name, e->notes[nnum].namelen);
	size += e->notes[nnum].namelen;
	aligned = elfc_align(size, sizeof(GElf_Word));
	while (size < aligned)
		buf[size++] = 0;
	memcpy(buf + size, e->notes[nnum].data, e->notes[nnum].datalen);
	size += e->notes[nnum].datalen;
	aligned = elfc_align(size, sizeof(GElf_Word));
	while (size < aligned)
		buf[size++] = 0;

	return size;
}

static int
put_elf64_note(struct elfc *e, int nnum, char *buf, size_t len)
{
	Elf64_Nhdr *nhdr = (Elf64_Nhdr *) buf;
	GElf_Off size, aligned;

	if (nnum > e->num_notes) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (len < sizeof(*nhdr)
	    + e->notes[nnum].namelen + e->notes[nnum].datalen) {
		e->eerrno = EINVAL;
		return -1;
	}

	nhdr->n_namesz = elfc_putWord(e, e->notes[nnum].namelen);
	nhdr->n_descsz = elfc_getWord(e, e->notes[nnum].datalen);
	nhdr->n_type = elfc_getWord(e, e->notes[nnum].type);
	size = sizeof(*nhdr);
	memcpy(buf + size, e->notes[nnum].name, e->notes[nnum].namelen);
	size += e->notes[nnum].namelen;
	aligned = elfc_align(size, sizeof(GElf_Word));
	while (size < aligned)
		buf[size++] = 0;
	memcpy(buf + size, e->notes[nnum].data, e->notes[nnum].datalen);
	size += e->notes[nnum].datalen;
	aligned = elfc_align(size, sizeof(GElf_Word));
	while (size < aligned)
		buf[size++] = 0;

	return size;
}

static int
put_elf_note(struct elfc *e, int nnum, char *buf, size_t len)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return put_elf32_note(e, nnum, buf, len);
	else
		return put_elf64_note(e, nnum, buf, len);
}

int
elfc_get_num_notes(struct elfc *e)
{
	if (!e->notes && (e->fd != -1)) {
		int rv = elfc_read_notes(e);
		if (rv == -1)
			return rv;
	}

	return e->num_notes;
}

int
elfc_get_note(struct elfc *e, int index,
	      GElf_Word *type,
	      const char **name, size_t *namelen,
	      const void **data, size_t *datalen)
{
	int rv;

	if (!e->notes && (e->fd != -1)) {
		rv = elfc_read_notes(e);
		if (rv == -1)
			return rv;
	}

	if (index > e->num_notes) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (type)
		*type = e->notes[index].type;
	if (name)
		*name = e->notes[index].name;
	if (namelen)
		*namelen = e->notes[index].namelen;
	if (data)
		*data = e->notes[index].data;
	if (datalen)
		*datalen = e->notes[index].datalen;
	return 0;
}

int
elfc_set_note_data(struct elfc *e, int index,
		   int set_type, GElf_Word type,
		   const char *name, size_t namelen,
		   const void *data, size_t datalen)
{
	int rv;
	char *newname = NULL;
	char *newdata = NULL;

	if (!e->notes && (e->fd != -1)) {
		rv = elfc_read_notes(e);
		if (rv == -1)
			return rv;
	}

	if (index > e->num_notes) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (set_type)
		e->notes[index].type = type;
	if (name) {
		newname = malloc(namelen + 1);
		if (!newname) {
			e->eerrno = ENOMEM;
			goto out_err;
		}
		memcpy(newname, name, namelen);
		newname[namelen] = '\0';
	}
	if (data) {
		newdata = malloc(datalen);
		if (!newdata) {
			e->eerrno = ENOMEM;
			goto out_err;
		}
		memcpy(newdata, data, datalen);
	}
	if (name) {
		free(e->notes[index].name);
		e->notes[index].name = newname;
		e->notes[index].namelen = namelen;
	}
	if (data) {
		free(e->notes[index].data);
		e->notes[index].data = newdata;
		e->notes[index].datalen = datalen;
	}

	return 0;

out_err:
	if (newname)
		free(newname);
	if (newdata)
		free(newdata);
	return -1;
}

int
elfc_del_note(struct elfc *e, int index)
{
	int rv;
	int i;

	if (!e->notes && (e->fd != -1)) {
		rv = elfc_read_notes(e);
		if (rv == -1)
			return rv;
	}

	if (index > e->num_notes) {
		e->eerrno = EINVAL;
		return -1;
	}

	free(e->notes[index].name);
	free(e->notes[index].data);
	e->num_notes--;
	for (i = index; i < e->num_notes; i++)
		e->notes[i] = e->notes[i + 1];

	return 0;
}

static void
free_phdrs(struct elfc *e)
{
	int i;

	phdr_phys_free(&e->phdr_tree);

	if (!e->phdrs)
		return;
	for (i = 0; i < e->num_phdrs; i++) {
		if (e->phdrs[i]->data_free)
			e->phdrs[i]->data_free(e, e->phdrs[i]->data,
					       e->phdrs[i]->userdata);
		free(e->phdrs[i]);
	}
	free(e->phdrs);
	e->phdrs = NULL;
	e->num_phdrs = 0;
	e->alloced_phdrs = 0;
}

static int
write_elf32_phdrs(struct elfc *e)
{
	int i;
	int rv;
	Elf32_Phdr *p32;
	size_t l = sizeof(*p32) * e->num_phdrs;

	p32 = malloc(l);
	if (!p32) {
		e->eerrno = ENOMEM;
		return -1;
	}
	for (i = 0; i < e->num_phdrs; i++) {
#define PhdrE(type, name) p32[i].p_ ## name = \
	elfc_put ## type(e, e->phdrs[i]->p.p_ ## name)
		Phdr32_Entries;
#undef PhdrE
	}
	rv = write(e->fd, p32, l);
	free(p32);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != l) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static int
write_elf64_phdrs(struct elfc *e)
{
	int i;
	int rv;
	Elf64_Phdr *p64;
	size_t l = sizeof(*p64) * e->num_phdrs;

	p64 = malloc(l);
	if (!p64) {
		e->eerrno = ENOMEM;
		return -1;
	}
	for (i = 0; i < e->num_phdrs; i++) {
#define PhdrE(type, name) p64[i].p_ ## name = \
	elfc_put ## type(e, e->phdrs[i]->p.p_ ## name)
		Phdr64_Entries;
#undef PhdrE
	}
	rv = write(e->fd, p64, l);
	free(p64);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != l) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static int
elfc_write_phdrs(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return write_elf32_phdrs(e);
	else
		return write_elf64_phdrs(e);
}

static int
read_elf32_phdrs(struct elfc *e, char *buf, GElf_Word phnum)
{
	int i, rv;
	struct elfc_phdr **phdrs;

	phdrs = malloc(sizeof(*phdrs) * phnum);
	if (!phdrs) {
		e->eerrno = ENOMEM;
		return -1;
	}

	e->num_phdrs = 0;
	e->alloced_phdrs = phnum;
	e->phdrs = phdrs;

	for (i = 0; i < phnum; i++) {
		struct elfc_phdr *p = malloc(sizeof(*p));
		Elf32_Phdr *p32 = ((Elf32_Phdr *)
				   (buf + (i * e->hdr.e_phentsize)));;

		if (!p) {
			e->eerrno = ENOMEM;
			free_phdrs(e);
			return -1;
		}

#define PhdrE(type, name) p->p.p_ ## name =			\
				elfc_get ## type(e, p32->p_ ## name)
		Phdr32_Entries;
#undef PhdrE

		if (p->p.p_type == PT_LOAD) {
			rv = phdr_phys_add(&e->phdr_tree, p);
			if (rv) {
				free(p);
				free_phdrs(e);
				e->eerrno = ENOMEM;
				return -1;
			}
		}
		e->phdrs[i] = p;
		e->num_phdrs++;
	}

	return 0;
}

static int
read_elf64_phdrs(struct elfc *e, char *buf, GElf_Word phnum)
{
	int i, rv;
	struct elfc_phdr **phdrs;

	phdrs = malloc(sizeof(*phdrs) * phnum);
	if (!phdrs) {
		e->eerrno = ENOMEM;
		return -1;
	}

	e->num_phdrs = 0;
	e->alloced_phdrs = phnum;
	e->phdrs = phdrs;

	for (i = 0; i < phnum; i++) {
		struct elfc_phdr *p = malloc(sizeof(*p));
		Elf64_Phdr *p64 = ((Elf64_Phdr *)
				   (buf + (i * e->hdr.e_phentsize)));

		if (!p) {
			e->eerrno = ENOMEM;
			free_phdrs(e);
			return -1;
		}

#define PhdrE(type, name) p->p.p_ ## name = \
				elfc_get ## type(e, p64->p_ ## name)
		Phdr64_Entries;
#undef PhdrE

		if (p->p.p_type == PT_LOAD) {
			rv = phdr_phys_add(&e->phdr_tree, p);
			if (rv) {
				free(p);
				free_phdrs(e);
				e->eerrno = ENOMEM;
				return -1;
			}
		}
		e->phdrs[i] = p;
		e->num_phdrs++;
	}

	return 0;
}

static int get_phnum(struct elfc *e, GElf_Word *r_phnum)
{
	GElf_Word phnum = e->hdr.e_phnum;

	if (phnum == PN_XNUM) {
		/* It's in section 0 sh_info field. */
		if (e->hdr.e_shnum == 0) {
			e->eerrno = EINVAL;
			return -1;
		}
		phnum = e->shdrs[0].sh.sh_info;
	}

	*r_phnum = phnum;
	return 0;
}

static int
elfc_read_phdrs(struct elfc *e)
{
	void *buf;
	int rv;
	GElf_Word phnum;
	void *tmpdata = NULL;

	if (e->phdrs) {
		e->eerrno = EBUSY;
		return -1;
	}

	rv = get_phnum(e, &phnum);
	if (rv == -1)
		return -1;

	rv = elfc_alloc_read_data(e, e->hdr.e_phoff, &buf,
				  e->hdr.e_phentsize * phnum);
	if (rv == -1)
		return -1;

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		rv = read_elf32_phdrs(e, buf, phnum);
	else
		rv = read_elf64_phdrs(e, buf, phnum);
	free(buf);
	if (rv != -1) {
		int i;

		for (i = 0; i < e->num_phdrs; i++) {
			e->phdrs[i]->userdata = elfc_tmpfile_alloc(e, &tmpdata);
			if (!e->phdrs[i]->userdata) {
				e->eerrno = ENOMEM;
				return -1;
			}
			e->phdrs[i]->pre_write = elfc_phdr_tmpfile_pre_write;
			e->phdrs[i]->do_write = elfc_phdr_tmpfile_do_write;
			e->phdrs[i]->post_write = elfc_phdr_tmpfile_post_write;
			e->phdrs[i]->data_free = elfc_phdr_tmpfile_free;
			e->phdrs[i]->get_data = elfc_phdr_tmpfile_get_data;
			e->phdrs[i]->set_data = elfc_phdr_tmpfile_set_data;
		}
	}
	return rv;
}

static void
free_shdrs(struct elfc *e)
{
	int i;

	if (!e->shdrs)
		return;
	for (i = 0; i < e->num_shdrs; i++) {
		if (e->shdrs[i].data_free)
			e->shdrs[i].data_free(e, e->shdrs[i].data,
					      e->shdrs[i].userdata);
	}
	free(e->shdrs);
	e->shdrs = NULL;
	e->num_shdrs = 0;
	e->alloced_shdrs = 0;
}

static int
write_elf32_shdrs(struct elfc *e)
{
	int i;
	int rv;
	Elf32_Shdr *p32;
	size_t l = sizeof(*p32) * e->num_shdrs;

	p32 = malloc(l);
	if (!p32) {
		e->eerrno = ENOMEM;
		return -1;
	}
	for (i = 0; i < e->num_shdrs; i++) {
#define ShdrE(type, name) p32[i].sh_ ## name = \
	elfc_put ## type(e, e->shdrs[i].sh.sh_ ## name)
		Shdr32_Entries;
#undef ShdrE
	}
	rv = write(e->fd, p32, l);
	free(p32);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != l) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static int
write_elf64_shdrs(struct elfc *e)
{
	int i;
	int rv;
	Elf64_Shdr *p64;
	size_t l = sizeof(*p64) * e->num_shdrs;

	p64 = malloc(l);
	if (!p64) {
		e->eerrno = ENOMEM;
		return -1;
	}
	for (i = 0; i < e->num_shdrs; i++) {
#define ShdrE(type, name) p64[i].sh_ ## name = \
	elfc_put ## type(e, e->shdrs[i].sh.sh_ ## name)
		Shdr64_Entries;
#undef ShdrE
	}
	rv = write(e->fd, p64, l);
	free(p64);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != l) {
		e->eerrno = EINVAL;
		return -1;
	}
	return 0;
}

static int
elfc_write_shdrs(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return write_elf32_shdrs(e);
	else
		return write_elf64_shdrs(e);
}

static int
read_elf32_shdrs(struct elfc *e, char *buf)
{
	int i;
	struct elfc_shdr *shdrs;

	shdrs = malloc(sizeof(*shdrs) * e->hdr.e_shnum);
	if (!shdrs) {
		e->eerrno = ENOMEM;
		return -1;
	}

	e->num_shdrs = e->hdr.e_shnum;
	e->alloced_shdrs = e->hdr.e_shnum;
	e->shdrs = shdrs;

	for (i = 0; i < e->num_shdrs; i++) {
		Elf32_Shdr *p32 = ((Elf32_Shdr *)
				   (buf + (i * e->hdr.e_shentsize)));;
#define ShdrE(type, name) e->shdrs[i].sh.sh_ ## name = \
	elfc_get ## type(e, p32->sh_ ## name)
		Shdr32_Entries;
#undef ShdrE
	}

	return 0;
}

static int
read_elf64_shdrs(struct elfc *e, char *buf)
{
	int i;
	struct elfc_shdr *shdrs;

	shdrs = malloc(sizeof(*shdrs) * e->hdr.e_shnum);
	if (!shdrs) {
		e->eerrno = ENOMEM;
		return -1;
	}

	e->num_shdrs = e->hdr.e_shnum;
	e->alloced_shdrs = e->hdr.e_shnum;
	e->shdrs = shdrs;

	for (i = 0; i < e->num_shdrs; i++) {
		Elf64_Shdr *p64 = ((Elf64_Shdr *)
				   (buf + (i * e->hdr.e_shentsize)));

#define ShdrE(type, name) e->shdrs[i].sh.sh_ ## name = \
	elfc_get ## type(e, p64->sh_ ## name)
		Shdr64_Entries;
#undef ShdrE
	}

	return 0;
}

static int
elfc_read_shdrs(struct elfc *e)
{
	void *buf;
	int rv;
	Elf32_Word i;
	const char *name;
	void *tmpdata = NULL;

	if (e->shdrs) {
		e->eerrno = EBUSY;
		return -1;
	}

	rv = elfc_alloc_read_data(e, e->hdr.e_shoff, &buf,
				  e->hdr.e_shentsize * e->hdr.e_shnum);
	if (rv == -1)
		return -1;

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		rv = read_elf32_shdrs(e, buf);
	else
		rv = read_elf64_shdrs(e, buf);
	free(buf);
	if (rv == -1)
		return -1;

	for (i = 0; i < e->num_shdrs; i++) {
		e->shdrs[i].userdata = elfc_tmpfile_alloc(e, &tmpdata);
		if (!e->shdrs[i].userdata) {
			e->eerrno = ENOMEM;
			return -1;
		}
		e->shdrs[i].data = NULL;
		e->shdrs[i].pre_write = elfc_shdr_tmpfile_pre_write;
		e->shdrs[i].do_write = elfc_shdr_tmpfile_do_write;
		e->shdrs[i].post_write = elfc_shdr_tmpfile_post_write;
		e->shdrs[i].data_free = elfc_shdr_tmpfile_free;
		e->shdrs[i].get_data = elfc_shdr_tmpfile_get_data;
		e->shdrs[i].set_data = elfc_shdr_tmpfile_set_data;
	}

	e->shstrtab = e->hdr.e_shstrndx;
	if (e->shstrtab == 0)
		/* No use looking for other sections. */
		goto no_shstrtab;

	if (e->shstrtab >= e->num_shdrs) {
		e->eerrno = EINVAL;
		return -1;
	}

	/* Validate .shstrtab. */
	if (e->shdrs[e->shstrtab].sh.sh_type != SHT_STRTAB) {
		e->eerrno = EINVAL;
		return -1;
	}
	name = elfc_get_shstr(e, e->shdrs[e->shstrtab].sh.sh_name);
	if (!name || strcmp(name, ".shstrtab") != 0) {
		e->eerrno = EINVAL;
		return -1;
	}

	/* Now look for .strtab and .symtab. */
	for (i = 0; i < e->num_shdrs; i++) {
		if (e->shdrs[i].sh.sh_type == SHT_STRTAB) {
			if (e->strtab || i == e->shstrtab)
				continue;
		} else if (e->shdrs[i].sh.sh_type == SHT_SYMTAB) {
			if (e->symtab)
				continue;
		} else
			continue;

		name = elfc_get_shstr(e, e->shdrs[i].sh.sh_name);

		if (e->shdrs[i].sh.sh_type == SHT_STRTAB &&
		    name && strcmp(name, ".strtab") == 0)
			e->strtab = i;
		else if (e->shdrs[i].sh.sh_type == SHT_SYMTAB &&
			 name && strcmp(name, ".symtab") == 0)
			e->symtab = i;
	}

	if (e->symtab) {
		if (e->shdrs[e->symtab].sh.sh_entsize !=
		    elfc_sym_size_one(e)) {
			e->eerrno = EINVAL;
			return -1;
		}
		e->symtab_size = e->shdrs[e->symtab].sh.sh_size /
			elfc_sym_size_one(e);
	}
no_shstrtab:
	return rv;
}

Elf32_Word
elfc_sym_size_one(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return sizeof(Elf32_Sym);
	else
		return sizeof(Elf64_Sym);
}

Elf32_Word
elfc_num_syms(struct elfc *e)
{
	return e->symtab_size;
}

static const char *
elfc_get_genstr(struct elfc *e, struct elfc_shdr *shstrsect,
		Elf32_Word shindex, Elf32_Word index)
{
	char *strdata;

	if (index == 0 || index >= shstrsect->sh.sh_size) {
		e->eerrno = EINVAL;
		return NULL;
	}
	if (!shstrsect->data) {
		void *odata;
		if (elfc_shdr_alloc_read(e, shindex, 0, &odata,
					 shstrsect->sh.sh_size) == -1) {
			return NULL;
		}
		strdata = odata;

		/* Make user it ends in a nil. */
		if (strdata[shstrsect->sh.sh_size - 1] != '\0') {
			free(odata);
			e->eerrno = EINVAL;
			return NULL;
		}
		/*
		 * Since "data" is unused for these types of section
		 * headers, install our own data there.
		 */
		shstrsect->data = odata;
	} else {
		strdata = shstrsect->data;
	}

	if (strdata[index - 1] != '\0') {
		/* Every string must have a nil before it. */
		e->eerrno = EINVAL;
		return NULL;
	}

	return strdata + index;
}

const char *
elfc_get_shstr(struct elfc *e, Elf32_Word index)
{
	if (e->shstrtab == 0) {
		e->eerrno = ENOENT;
		return NULL;
	}
	return elfc_get_genstr(e, &e->shdrs[e->shstrtab], e->shstrtab, index);
}

const char *
elfc_get_str(struct elfc *e, Elf32_Word index)
{
	if (e->strtab == 0) {
		e->eerrno = ENOENT;
		return NULL;
	}
	return elfc_get_genstr(e, &e->shdrs[e->strtab], e->strtab, index);
}

static struct elfc_shdr *
elfc_check_sym(struct elfc *e, Elf32_Word index)
{
	struct elfc_shdr *symsect;

	if (e->symtab == 0) {
		e->eerrno = ENOENT;
		return NULL;
	}

	if (index == 0 || index >= e->symtab_size) {
		e->eerrno = EINVAL;
		return NULL;
	}
	symsect = &e->shdrs[e->symtab];

	if (!symsect->data) {
		void *odata;
		if (elfc_shdr_alloc_read(e, e->symtab, 0, &odata,
					 symsect->sh.sh_size) == -1) {
			return NULL;
		}
		/*
		 * Since "data" is unused for these types of section
		 * headers, install our own data there.
		 */
		symsect->data = odata;
	}

	return symsect;
}

#define Sym32_Entries \
	ShdrE(Word,	name);		\
	ShdrE(Addr,	value);		\
	ShdrE(Word,	size);		\
	ShdrE(uchar,	info);		\
	ShdrE(uchar,	other);		\
	ShdrE(Section,	shndx);

#define Sym64_Entries \
	ShdrE(Word,	name);		\
	ShdrE(uchar,	info);		\
	ShdrE(uchar,	other);		\
	ShdrE(Section,	shndx);		\
	ShdrE(Addr,	value);		\
	ShdrE(Word,	size);

static void
read_elf32_sym(struct elfc *e, struct elfc_shdr *symsect,
	       Elf32_Word index, GElf_Sym *rsym)
{
	Elf32_Sym *sym;

	sym = symsect->data;
	sym += index;

#define ShdrE(type, name) rsym->st_ ## name = \
	elfc_get ## type(e, sym->st_ ## name)
		Sym32_Entries;
#undef ShdrE
}

static void
read_elf64_sym(struct elfc *e, struct elfc_shdr *symsect,
	       Elf32_Word index, GElf_Sym *rsym)
{
	Elf64_Sym *sym;

	sym = symsect->data;
	sym += index;

#define ShdrE(type, name) rsym->st_ ## name = \
	elfc_get ## type(e, sym->st_ ## name)
		Sym64_Entries;
#undef ShdrE
}

int
elfc_get_sym(struct elfc *e, Elf32_Word index, GElf_Sym *sym)
{
	struct elfc_shdr *symsect;

	symsect = elfc_check_sym(e, index);
	if (!symsect)
		return -1;

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		read_elf32_sym(e, symsect, index, sym);
	else
		read_elf64_sym(e, symsect, index, sym);

	return 0;
}

static void
write_elf32_sym(struct elfc *e, struct elfc_shdr *symsect,
		Elf32_Word index, GElf_Sym *rsym)
{
	Elf32_Sym *sym;

	sym = symsect->data;
	sym += index;

#define ShdrE(type, name) sym->st_ ## name = \
	elfc_put ## type(e, rsym->st_ ## name)
		Sym32_Entries;
#undef ShdrE
}

static void
write_elf64_sym(struct elfc *e, struct elfc_shdr *symsect,
		Elf32_Word index, GElf_Sym *rsym)
{
	Elf64_Sym *sym;

	sym = symsect->data;
	sym += index;

#define ShdrE(type, name) sym->st_ ## name = \
	elfc_put ## type(e, rsym->st_ ## name)
		Sym64_Entries;
#undef ShdrE
}

int
elfc_set_sym(struct elfc *e, Elf32_Word index, GElf_Sym *sym)
{
	struct elfc_shdr *symsect;

	symsect = elfc_check_sym(e, index);
	if (!symsect)
		return -1;

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		write_elf32_sym(e, symsect, index, sym);
	else
		write_elf64_sym(e, symsect, index, sym);

	return 0;
}

static Elf32_Word
read_elf32_sym_name(struct elfc *e, struct elfc_shdr *symsect,
		    Elf32_Word index)
{
	Elf32_Sym *sym;

	sym = symsect->data;
	sym += index;

	return elfc_getWord(e, sym->st_name);
}

static int
read_elf64_sym_name(struct elfc *e, struct elfc_shdr *symsect,
		    Elf32_Word index)
{
	Elf64_Sym *sym;

	sym = symsect->data;
	sym += index;

	return elfc_getWord(e, sym->st_name);
}

const char *
elfc_get_sym_name(struct elfc *e, Elf32_Word index)
{
	Elf32_Word stridx;
	struct elfc_shdr *symsect;

	symsect = elfc_check_sym(e, index);
	if (!symsect)
		return NULL;

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		stridx = read_elf32_sym_name(e, symsect, index);
	else
		stridx = read_elf64_sym_name(e, symsect, index);

	return elfc_get_str(e, stridx);
}

int
elfc_lookup_sym(struct elfc *e, const char *name, GElf_Sym *sym,
	        Elf32_Word startidx, Elf32_Word *symidx)
{
	Elf32_Word nsyms = elfc_num_syms(e);
	Elf32_Word i;
	const char *tname;

	for (i = startidx + 1; i < nsyms; i++) {
		tname = elfc_get_sym_name(e, i);
		if (tname && strcmp(tname, name) == 0)
			goto found;
	}
	e->eerrno = ENOENT;
	return -1;

found:
	if (sym)
		elfc_get_sym(e, i, sym);
	if (symidx)
		*symidx = i;
	return 0;
}

struct elfc *
elfc_alloc(void)
{
	struct elfc *e;

	e = malloc(sizeof(*e));
	if (!e)
		return NULL;
	memset(e, 0, sizeof(*e));
	e->fd = -1;
	if (phdr_phys_init(&e->phdr_tree)) {
	    free(e);
	    return NULL;
	}
	e->phdr_tree.Allow_Duplicates = 1;
	return e;
}

int
elfc_setup(struct elfc *e, GElf_Half type)
{
	if (!elfclass) {
		if (sizeof(char *) == 4)
			elfclass = ELFCLASS32;
		else
			elfclass = ELFCLASS64;
	}

	memset(&e->hdr, 0, sizeof(e->hdr));
	e->hdr.e_ident[EI_MAG0] = ELFMAG0;
	e->hdr.e_ident[EI_MAG1] = ELFMAG1;
	e->hdr.e_ident[EI_MAG2] = ELFMAG2;
	e->hdr.e_ident[EI_MAG3] = ELFMAG3;
	e->hdr.e_ident[EI_CLASS] = elfclass;
	e->hdr.e_ident[EI_DATA] = elfendian;
	e->hdr.e_ident[EI_VERSION] = EV_CURRENT;
	e->hdr.e_ident[EI_OSABI] = ELFOSABI_NONE;
	e->hdr.e_ident[EI_ABIVERSION] = 0;
	e->hdr.e_type = type;
	e->hdr.e_machine = elfarch;
	e->hdr.e_version = EV_CURRENT;
	return 0;
}

static void
elfc_freei(struct elfc *e)
{
	free_shdrs(e);
	free_phdrs(e);
	free_notes(e);
}

void
elfc_free(struct elfc *e)
{
	elfc_freei(e);
	free(e);
}

static int
validate_elf_header(struct elfc *e)
{
	GElf_Word phnum;

	if (e->hdr.e_phoff < e->hdr.e_ehsize)
		return -1;
	if (e->hdr.e_phentsize < elfc_phdr_size_one(e))
		return -1;
	if (e->hdr.e_shoff && e->hdr.e_shoff < e->hdr.e_ehsize)
		return -1;
	if (e->hdr.e_shoff && e->hdr.e_shentsize < elfc_shdr_size_one(e))
		return -1;
	if (get_phnum(e, &phnum) == -1)
		return -1;

	e->after_headers = e->hdr.e_phoff +
		(e->hdr.e_phentsize * phnum) +
		(e->hdr.e_shentsize * e->hdr.e_shnum);
	return 0;
}

int
elfc_open(struct elfc *e, int fd)
{
	int rv;
	size_t l;

	if (e->shdrs) {
	    /* Already opened. */
	    e->eerrno = EBUSY;
	    return -1;
	}

	rv = lseek(fd, 0, SEEK_SET);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	l = sizeof(e->hdr.e_ident);
	rv = read(fd, &e->hdr, l);
	if (rv == -1) {
		e->eerrno = errno;
		return -1;
	}
	if (rv != l) {
		e->eerrno = EINVAL;
		return -1;
	}

	if (memcmp(e->hdr.e_ident, ELFMAG, SELFMAG) != 0) {
		e->eerrno = EINVAL;
		return -1;
	}
	switch (e->hdr.e_ident[EI_CLASS]) {
	case ELFCLASS32:
	case ELFCLASS64:
		break;
	default:
		e->eerrno = EINVAL;
		return -1;
	}

	switch (e->hdr.e_ident[EI_DATA]) {
	case ELFDATA2LSB:
	case ELFDATA2MSB:
		break;
	default:
		e->eerrno = EINVAL;
		return -1;
	}

	if (e->hdr.e_ident[EI_VERSION] != EV_CURRENT) {
		e->eerrno = EINVAL;
		return -1;
	}

	e->fd = fd;
	switch (e->hdr.e_ident[EI_CLASS]) {
	case ELFCLASS32:
		rv = read_elf32_ehdr(e);
		if (rv == -1)
			return rv;
		break;
	case ELFCLASS64:
		rv = read_elf64_ehdr(e);
		break;
	}
	if (rv)
		goto out;

	rv = elfc_read_shdrs(e);
	if (rv == -1)
		goto out;

	rv = validate_elf_header(e);
	if (rv == -1) {
		e->eerrno = EINVAL;
		goto out;
	}

	rv = elfc_read_phdrs(e);
out:
	return rv;
}

GElf_Off
elfc_ehdr_size(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return sizeof(Elf32_Ehdr);
	else
		return sizeof(Elf64_Ehdr);
}

GElf_Off
elfc_shdr_size_one(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return sizeof(Elf32_Shdr);
	else
		return sizeof(Elf64_Shdr);
}

GElf_Off
elfc_shdr_size(struct elfc *e)
{
	GElf_Off size;
	size = elfc_shdr_size_one(e) * e->hdr.e_shnum;
	if ((e->num_shdrs > 65534) &&
	    ((e->hdr.e_shnum == 0) || (e->shdrs[0].sh.sh_type != SHT_NULL)))
		size += elfc_shdr_size_one(e);
	return size;
}

GElf_Off
elfc_phdr_size_one(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return sizeof(Elf32_Phdr);
	else
		return sizeof(Elf64_Phdr);
}

GElf_Off
elfc_phdr_size(struct elfc *e)
{
	GElf_Off size;
	GElf_Word phnum = 0;

	get_phnum(e, &phnum);
	size = elfc_phdr_size_one(e) * phnum;
	if (e->notes)
		size += elfc_phdr_size_one(e);
	return size;
}

GElf_Off
elfc_nhdr_size_one(struct elfc *e)
{
	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		return sizeof(Elf32_Nhdr);
	else
		return sizeof(Elf64_Nhdr);
}

GElf_Off
elfc_notes_size(struct elfc *e)
{
	GElf_Off size = 0;
	int i;

	if (!e->notes && (e->fd != -1))
		elfc_read_notes(e);

	if (!e->num_notes)
		return 0;

	for (i = 0; i < e->num_notes; i++) {
		size += elfc_nhdr_size_one(e);
		size += e->notes[i].namelen;
		size = elfc_align(size, sizeof(GElf_Word));
		size += e->notes[i].datalen;
		size = elfc_align(size, sizeof(GElf_Word));
	}
	return size;
}

GElf_Off
elfc_headers_size(struct elfc *e)
{
	return elfc_ehdr_size(e) + elfc_phdr_size(e) + elfc_shdr_size(e);
}

GElf_Off
elfc_data_offset_start(struct elfc *e)
{
	return elfc_headers_size(e) + elfc_notes_size(e);
}

static void
call_phdr_post_write(struct elfc *e, int i)
{
	if (e->phdrs[i]->post_write)
		e->phdrs[i]->post_write(e, &e->phdrs[i]->p,
					e->phdrs[i]->data,
					e->phdrs[i]->userdata);
}

static void
call_shdr_post_write(struct elfc *e, int i)
{
	if (e->shdrs[i].post_write)
		e->shdrs[i].post_write(e, &e->shdrs[i].sh,
				       e->shdrs[i].data,
				       e->shdrs[i].userdata);
}

static int
isregfile(int fd)
{
	int rv;
	struct stat st;

	rv = fstat(fd, &st);
	if (rv == -1)
		return -1;
	return S_ISREG(st.st_mode);
}

int
elfc_write(struct elfc *e)
{
	int rv;
	int i;
	GElf_Off off;

	if (e->notes) {
		/* Insert a new phdr for the notes then free the notes. */
		GElf_Off nsize = elfc_notes_size(e);
		size_t pos = 0;
		char *ndata = malloc(nsize);

		if (!ndata) {
			e->eerrno = ENOMEM;
			return -1;
		}
		for (i = 0; i < e->num_notes; i++) {
			rv = put_elf_note(e, i, ndata + pos, nsize - pos);
			if (rv == -1) {
				free(ndata);
				return -1;
			}
			pos += rv;
		}
		rv = elfc_insert_phdr(e, 0, PT_NOTE, 0, 0, nsize, 0,
				      0, 0);
		if (rv == -1) {
			free(ndata);
			return -1;
		}
		elfc_set_phdr_data(e, rv, ndata, elfc_gen_phdr_free,
				   NULL, elfc_phdr_block_do_write, NULL,
				   elfc_phdr_block_get_data,
				   elfc_phdr_block_set_data, NULL);
		free_notes(e);
	}

	off = elfc_ehdr_size(e);
	e->hdr.e_ehsize = off;
	if (e->num_phdrs > 65534) {
		e->hdr.e_phnum = PN_XNUM;
		/*
		 * We use a dummy NULL section header in section 0 to
		 * hold the number of entries in sh_info.
		 */
		if ((e->hdr.e_shnum == 0)
		    || (e->shdrs[0].sh.sh_type != SHT_NULL)) {
			rv = elfc_insert_shdr(e, 0, SHN_UNDEF, SHT_NULL,
					      0, 0, 0, 0,
					      0, e->num_phdrs, 0, 0);
			if (rv == -1)
				return rv;
		} else {
			e->shdrs[0].sh.sh_info = e->num_phdrs;
		}
	} else {
		e->hdr.e_phnum = e->num_phdrs;
	}

	e->hdr.e_shnum = e->num_shdrs;
	if (e->hdr.e_shnum) {
		e->hdr.e_shentsize = elfc_shdr_size_one(e);
		e->hdr.e_shoff = off;
		off += elfc_shdr_size(e);
	}

	if (e->num_phdrs) {
		e->hdr.e_phoff = off;
		e->hdr.e_phentsize = elfc_phdr_size_one(e);
		off += elfc_phdr_size(e);
	}

	/*
	 * Do pre-write before reset the file so that we are
	 * still working with the original offsets when we save off
	 * the information.
	 */
	for (i = 0; i < e->num_shdrs; i++) {
		if (e->shdrs[i].pre_write) {
			rv = e->shdrs[i].pre_write(e, &e->shdrs[i].sh,
						   e->shdrs[i].data,
						   e->shdrs[i].userdata);
			if (rv == -1) {
				e->eerrno = errno;
				i--;
				for (; i > 0; i--)
					call_shdr_post_write(e, i);
				goto out;
			}
		}
	}

	for (i = 0; i < e->num_phdrs; i++) {
		if (e->phdrs[i]->pre_write) {
			rv = e->phdrs[i]->pre_write(e, &e->phdrs[i]->p,
						    e->phdrs[i]->data,
						    e->phdrs[i]->userdata);
			if (rv == -1) {
				e->eerrno = errno;
				i--;
				for (; i > 0; i--)
					call_phdr_post_write(e, i);
				goto out;
			}
		}
	}

	/* Clear out the old data now that it should be saved. */
	rv = isregfile(e->fd);
	if (rv == -1) {
		e->eerrno = errno;
		goto out;
	} else if (rv) {
		rv = lseek(e->fd, 0, SEEK_SET);
		if (rv != -1)
			rv = ftruncate(e->fd, 0);
		if (rv == -1) {
			e->eerrno = errno;
			goto out;
		}
	}

	if (e->hdr.e_ident[EI_CLASS] == ELFCLASS32)
		rv = write_elf32_ehdr(e);
	else
		rv = write_elf64_ehdr(e);
	if (rv == -1)
		goto out;

	off = elfc_headers_size(e);
	for (i = 0; i < e->num_shdrs; i++) {
		e->shdrs[i].sh.sh_offset = off;
		off += e->shdrs[i].sh.sh_size;
	}

	for (i = 0; i < e->num_phdrs; i++) {
		e->phdrs[i]->p.p_offset = off;
		off += e->phdrs[i]->p.p_filesz;
	}

	rv = elfc_write_shdrs(e);
	if (rv == -1)
		goto out;

	rv = elfc_write_phdrs(e);
	if (rv == -1)
		goto out;

	for (i = 0; i < e->num_phdrs; i++) {
		if (e->phdrs[i]->do_write) {
			/*
			 * Should already be in the correct position
			 * here, no need to seek.
			 */
			rv = e->phdrs[i]->do_write(e, e->fd, &e->phdrs[i]->p,
						   e->phdrs[i]->data,
						   e->phdrs[i]->userdata);
			if (rv == -1) {
				e->eerrno = errno;
				goto out;
			}
		}
	}

out:
	for (i = 0; i < e->num_phdrs; i++)
		call_phdr_post_write(e, i);

	return rv;
}

int
elfc_get_errno(struct elfc *e)
{
	return e->eerrno;
}

void
elfc_set_fd(struct elfc *e, int fd)
{
	e->fd = fd;
}

int
elfc_get_fd(struct elfc *e)
{
	return e->fd;
}

void
elfc_set_userdata(struct elfc *e, void *userdata)
{
	e->userdata = userdata;
}

void *
elfc_get_userdata(struct elfc *e)
{
	return e->userdata;
}
