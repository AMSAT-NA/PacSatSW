/*
 * btree.h
 *
 * A btree "generic" or "template" in C.
 *
 * (C) 2003 MontaVista Software, Inc.  All right reserved.
 * 
 * This program is licensed under the MontaVista Software,
 * Inc. License Agreement ("License Agreement"), and is for the
 * purposes of the License Agreement a MontaVista Licensed Deployment
 * Program.  The License requires that you have a valid Product
 * Subscription with MontaVista Software, Inc., or are a Named Contact
 * with active access to the MontaVista Zone, or have a Software
 * License Agreement with MontaVista Software, Inc. This program comes
 * with no warranties other than those provided for in the Product
 * Subscription agreement. The License Agreement grants you the right
 * to install, modify and use the program.  You may distribute the
 * object code and scripts for this program, but you have no right to
 * distribute the source code for this program.
 */

/*
 * This is a btree C "generic", it allows you to define a btree for a
 * given type.  To use this, create a file with something like:
 *
 * #define BTREE_NODE_SIZE 10
 * typedef struct btree_val_s { int a; } btree_val_t;
 *
 * #define btree_t test_btree_t
 * #define BEN(s) test_ ## s
 * #define BTREE_NAMES_LOCAL static -- This is only if you want the symbols
 *			               defined here to be local only.
 *
 * int
 * btree_cmp_key(btree_val_t val1, btree_val_t val2)
 * {
 *     if (val1.a < val2.a) {
 * 	   return -1;
 *     } else if (val1.a > val2.a) {
 * 	   return 1;
 *     } else {
 * 	   return 0;
 *    }
 * }
 *
 * #include "btree.h"
 *
 * The node size of the btree needs to be set.  The btree_val_t type
 * is the type contained in the btree.  It needs to hold both the key
 * and the value you are interested in.  The type defined in btree.h
 * for the main btree is "btree_t", you can rename it if you like.
 * All the exported functions are named with BEN,
 * feel free to name them anything you like, but the value passed
 * in is what makes it unique.  btree_compare_key is the key comparison
 * function, it must return -1 if val1 < val2, 0 if val1 = val2, and
 * 1 if val1 > val2.
 *
 * Add a value to the btree.  The value must be unique.
 *
 * int
 * BEN(add) (btree_t     *tree,
 *           btree_val_t val)
 *
 *
 * Remove a value from the btree.  If the value is not the last value
 * in the tree, then next will be set to the value after the deleted
 * one and is_end will be set to false.  If the value is the last value
 * in the tree, is_end is set to true and the value of next is undefined.
 * Both next and is_end may be NULL, then the will be ignored.
 *
 * int
 * BEN(delete) (btree_t     *tree,
 * 		btree_val_t val,
 * 		btree_val_t *next,
 * 		int         *is_end)
 *
 *
 * Search for the given value in the btree.  When searching, the val
 * passed in only has to have the key information set.  The full value
 * is returned in "item".  closest_op may be BTREE_CLOSEST_NEXT, which
 * will return the item that would follow the given value in the true,
 * BTREE_CLOSEST_PREV which would return the previous value in the tree,
 * or BTREE_NO_CLOSEST, which will not set the item if not found.  Note
 * that BTREE_ITEM_NOT_FOUND will still be returned in these cases.
 * If BTREE_AT_END_OF_TREE is returned, then the given value would be
 * off the end of the tree.
 *
 * int
 * BEN(search) (btree_t     *tree,
 * 		btree_val_t val,
 *		btree_val_t *item,
 *              int         closest_op)
 *
 *
 * Get the first item in the btree.
 *
 * int
 * BEN(first) (btree_t     *tree,
 *	       btree_val_t *item)
 *
 *
 * Get the last item in the btree.
 *
 * int
 * BEN(last) (btree_t     *tree,
 *	      btree_val_t *item)
 *
 *
 * Get the next item in the btree.  If the val passed in is the last
 * item in the btree, then BTREE_END_OF_TREE is returned and next_item
 * is undefined.
 *
 * int
 * BEN(next) (btree_t     *tree,
 *	      btree_val_t val,
 *	      btree_val_t *next_item)
 *
 *
 * Get the previous item in the btree.  If the val passed in is the first
 * item in the btree, then BTREE_END_OF_TREE is returned and prev_item
 * is undefined.
 *
 * int
 * BEN(prev) (btree_t     *tree,
 *	      btree_val_t val,
 *	      btree_val_t *prev_item)
 *
 *
 * Initialize the btree.  This must be called before use of a btree.
 *
 * void
 * BEN(init) (btree_t *tree);
 *
 *
 * Free all the data in the btree.
 *
 * void
 * BEN(free) (btree_t *tree)
 */

/*
 * To avoid "unused function" errors, you only have to enable the
 * functions you need by setting BTREE_NEEDS with a bitmask of the
 * following.  By default all are set.  Note that you can't disable
 * add, free, init, or search, since it is assumed you need all those.
 */
#define BTREE_NEEDS_PREV	(1 << 0)
#define BTREE_NEEDS_NEXT	(1 << 1)
#define BTREE_NEEDS_LAST	(1 << 2)
#define BTREE_NEEDS_FIRST	(1 << 3)
#define BTREE_NEEDS_DELETE	(1 << 4)

/* Check to make sure the node size isn't too big. */
#if BTREE_NODE_SIZE > 255
#error BTREE_NODE_SIZE may not be larger than 255.
#endif

#ifndef BTREE_NEEDS
#define BTREE_NEEDS (BTREE_NEEDS_PREV | BTREE_NEEDS_NEXT | \
		     BTREE_NEEDS_LAST | BTREE_NEEDS_FIRST | \
		     BTREE_NEEDS_DELETE)
#endif

#ifndef BTREE_NAMES_LOCAL
#define BTREE_NAMES_LOCAL
#endif

/* When searching, this tells whether to return a closes next or previous
   value. */
#define BTREE_CLOSEST_NEXT 1
#define BTREE_NO_CLOSEST   0
#define BTREE_CLOSEST_PREV -1

/* The following are error return values from the btree calls.  Zero is
   returned when no error occurs. */
#define BTREE_ITEM_ALREADY_EXISTS	-1
#define BTREE_ITEM_NOT_FOUND		-2
#define BTREE_AT_END_OF_TREE		-3
#define BTREE_OUT_OF_MEMORY		-4

#include <stdlib.h>
#include <assert.h>

/* This code was lifted from an Ada95 Btree engine, that's why the
   coding style is so unusual, and there are no comments.  It's the
   Adasl Btree, see that for the actual comments. */

struct BEN(btree_node_s);
typedef struct BEN(btree_node_s) BEN(btree_node_t);
struct BEN(btree_leaf_s);
typedef struct BEN(btree_leaf_s) BEN(btree_leaf_t);
struct BEN(btree_nleaf_s);
typedef struct BEN(btree_nleaf_s) BEN(btree_nleaf_t);

struct BEN(btree_node_s)
{
    char         First;
    char         Last;
    char         Parent_Index;
    char         Leaf;
    BEN(btree_node_t) *Parent;
    btree_val_t  Vals[BTREE_NODE_SIZE];
};

struct BEN(btree_leaf_s)
{
    BEN(btree_node_t) node;
};

struct BEN(btree_nleaf_s)
{
    BEN(btree_node_t) node;
    BEN(btree_node_t) *(Children[BTREE_NODE_SIZE]);
    BEN(btree_node_t) *Right_Child;
};

static BEN(btree_leaf_t) *
BEN(container_of_leaf)(BEN(btree_node_t) *ptr)
{
    assert(ptr->Leaf);
    return ((BEN(btree_leaf_t) *)(((char *) ptr)
			- offsetof(BEN(btree_leaf_t), node)));
}

static BEN(btree_nleaf_t) *
BEN(container_of_nleaf)(BEN(btree_node_t) *ptr)
{
    assert(!ptr->Leaf);
    return ((BEN(btree_nleaf_t) *)(((char *) ptr)
			- offsetof(BEN(btree_nleaf_t), node)));
}

static BEN(btree_node_t) *
BEN(Get_Child)(BEN(btree_node_t) *node, int index)
{
    BEN(btree_nleaf_t) *nl = BEN(container_of_nleaf)(node);

    return nl->Children[index];
}

static void
BEN(Set_Child)(BEN(btree_node_t) *node, int index,
			     BEN(btree_node_t) *child)
{
    BEN(btree_nleaf_t) *nl = BEN(container_of_nleaf)(node);

    nl->Children[index] = child;
    child->Parent = node;
    child->Parent_Index = index;
}

static BEN(btree_node_t) *
BEN(Get_Right_Child)(BEN(btree_node_t) *node)
{
    BEN(btree_nleaf_t) *nl = BEN(container_of_nleaf)(node);

    return nl->Right_Child;
}

static void
BEN(Set_Right_Child)(BEN(btree_node_t) *node,
				   BEN(btree_node_t) *child)
{
    BEN(btree_nleaf_t) *nl = BEN(container_of_nleaf)(node);

    nl->Right_Child = child;
    child->Parent_Index = BTREE_NODE_SIZE;
    child->Parent = node;
}

typedef struct BEN(btree_s)
{
    BEN(btree_node_t)   *Root;
    unsigned int   Count;
    int            Allow_Duplicates;
    int            Update;
} btree_t;

#if BTREE_NEEDS & BTREE_NEEDS_DELETE
static int
BEN(Node_Count) (BEN(btree_node_t) *Node)
{
    if (Node->First <= Node->Last)
	return Node->Last - Node->First + 1;
    else
	return (BTREE_NODE_SIZE - Node->First) + Node->Last + 1;
}
#endif

static int
BEN(Node_Item_Pos) (BEN(btree_node_t) *Node,
		    int               Index)
{
    if (Node->First <= Index)
	return Index - Node->First;
    else
	return (BTREE_NODE_SIZE - Node->First) + Index;
}

static int
BEN(Next) (BEN(btree_node_t) *Node,
	   int               Curr)
{
    if (Curr < (BTREE_NODE_SIZE-1))
	return Curr + 1;
    else
	return 0;
}

static int
BEN(Prev) (BEN(btree_node_t) *Node,
	   int               Curr)
{
    if (Curr != 0)
	return Curr - 1;
    else
	return BTREE_NODE_SIZE-1;
}

#if BTREE_NEEDS & BTREE_NEEDS_FIRST
static void
BEN(Local_First) (btree_t          *tree,
		  BEN(btree_node_t) **Pos,
		  int               *Index,
		  int               *Is_End)
{
    BEN(btree_node_t) *Retval_Pos;

    if (tree->Count == 0)
	*Is_End = 1;
    else {
	*Is_End = 0;
	Retval_Pos = tree->Root;
	while (! Retval_Pos->Leaf) {
            Retval_Pos = BEN(Get_Child)(Retval_Pos, Retval_Pos->First);
	}
	*Pos = Retval_Pos;
        *Index = Retval_Pos->First;
    }
}
#endif

#if BTREE_NEEDS & BTREE_NEEDS_LAST
static void
BEN(Local_Last) (btree_t           *tree,
		 BEN(btree_node_t) **Pos,
		 int               *Index,
		 int               *Is_End)
{
    BEN(btree_node_t) *Retval_Pos;

    if (tree->Count == 0)
	*Is_End = 1;
    else {
	*Is_End = 0;
	Retval_Pos = tree->Root;
	while (! Retval_Pos->Leaf)
            Retval_Pos = Retval_Pos->Right_Child;
	*Pos = Retval_Pos;
        *Index = Retval_Pos->Last;
    }
}
#endif

static void
BEN(Local_Next) (btree_t           *tree,
		 BEN(btree_node_t) **Pos,
		 int               *Index,
		 int               *Is_End)
{
    BEN(btree_node_t) *Tmp_Pos;

    if (*Index != (*Pos)->Last) {
	if ((*Pos)->Leaf) {
	    *Is_End = 0;
            *Index = BEN(Next)(*Pos, *Index);
	} else {
            *Is_End = 0;
            Tmp_Pos = BEN(Get_Child)(*Pos, BEN(Next)(*Pos, *Index));
            while (! Tmp_Pos->Leaf) {
		Tmp_Pos = BEN(Get_Child)(Tmp_Pos, Tmp_Pos->First);
	    }
            *Pos = Tmp_Pos;
            *Index = Tmp_Pos->First;
	}
    } else {
	if ((*Pos)->Leaf) {
            Tmp_Pos = *Pos;
            while ((Tmp_Pos != tree->Root)
                   && (Tmp_Pos->Parent_Index == BTREE_NODE_SIZE))
		Tmp_Pos = Tmp_Pos->Parent;

            if (Tmp_Pos == tree->Root) {
		*Is_End = 1;
            } else {
		*Is_End = 0;
		*Index = Tmp_Pos->Parent_Index;
		*Pos = Tmp_Pos->Parent;
	    }
	} else {
            *Is_End = 0;
            Tmp_Pos = BEN(Get_Right_Child)(*Pos);
            while (! Tmp_Pos->Leaf) {
		Tmp_Pos = BEN(Get_Child)(Tmp_Pos, Tmp_Pos->First);
            }
            *Pos = Tmp_Pos;
            *Index = Tmp_Pos->First;
	}
    }
}


static void
BEN(Local_Prev) (btree_t           *tree,
		 BEN(btree_node_t) **Pos,
		 int               *Index,
		 int               *Is_End)
{
    BEN(btree_node_t) *Tmp_Pos;

    if (*Index != (*Pos)->First) {
	if ((*Pos)->Leaf) {
            *Is_End = 0;
            *Index = BEN(Prev)(*Pos, *Index);
	} else {
            *Is_End = 0;
            Tmp_Pos = BEN(Get_Child)(*Pos, BEN(Next)(*Pos, *Index));
            while (! Tmp_Pos->Leaf) {
	       Tmp_Pos = BEN(Get_Right_Child)(Tmp_Pos);
            }
            *Pos = Tmp_Pos;
            *Index = Tmp_Pos->Last;
         }
    } else {
         if ((*Pos)->Leaf) {
            Tmp_Pos = *Pos;
            while ((Tmp_Pos != tree->Root)
                   && (Tmp_Pos->Parent_Index == Tmp_Pos->Parent->First))
            {
               Tmp_Pos = Tmp_Pos->Parent;
            }

            if (Tmp_Pos == tree->Root) {
		*Is_End = 1;
            } else {
		*Is_End = 0;
		if (Tmp_Pos->Parent_Index == BTREE_NODE_SIZE) {
		    *Index = Tmp_Pos->Parent->Last;
		} else {
		    *Index = BEN(Prev)(Tmp_Pos->Parent, Tmp_Pos->Parent_Index);
		}
		*Pos = Tmp_Pos->Parent;
            }
         } else {
	     *Is_End = 0;
	     Tmp_Pos = BEN(Get_Child)(*Pos, (*Pos)->First);
	     while (! Tmp_Pos->Leaf) {
		 Tmp_Pos = BEN(Get_Right_Child)(Tmp_Pos);
	     }
	     *Pos = Tmp_Pos;
	     *Index = Tmp_Pos->Last;
         }
    }
}

static void
BEN(Local_Search) (btree_t           *tree,
		   btree_val_t       key,
		   BEN(btree_node_t) **Pos,
		   int               *Index,
		   int               *Found)
{
    BEN(btree_node_t) *Retval_Pos;
    int Retval_Index;
    int cmp_res;

    if (tree->Count == 0) {
	*Pos = NULL;
	*Found = 0;
	return;
    }

    Retval_Pos = tree->Root;
    Retval_Index = Retval_Pos->First;
    cmp_res = btree_cmp_key(Retval_Pos->Vals[Retval_Index], key);
    while (cmp_res != 0) {
	if (cmp_res > 0) {
            if (Retval_Pos->Leaf) {
		*Pos = Retval_Pos;
		*Index = Retval_Index;
		*Found = 0;
		return;
            }
            Retval_Pos = BEN(Get_Child)(Retval_Pos, Retval_Index);
            Retval_Index = Retval_Pos->First;
	} else if (Retval_Index == Retval_Pos->Last) {
            if (Retval_Pos->Leaf) {
		*Pos = Retval_Pos;
		*Index = Retval_Index;
		*Found = 0;
		return;
            }
            Retval_Pos = BEN(Get_Right_Child)(Retval_Pos);
            Retval_Index = Retval_Pos->First;
	} else {
            Retval_Index = BEN(Next)(Retval_Pos, Retval_Index);
	}
	cmp_res = btree_cmp_key(Retval_Pos->Vals[Retval_Index], key);
    }

    *Pos = Retval_Pos;
    *Index = Retval_Index;
    *Found = 1;
}

static BEN(btree_node_t) *
BEN(Left_Node) (btree_t           *tree,
		BEN(btree_node_t) *Pos)
{
    BEN(btree_node_t) *Retval;
    int Prev_Index;

    if (Pos->Parent == NULL) {
	Retval = NULL;
    } else if (Pos->Parent_Index == Pos->Parent->First) {
	Retval = NULL;
    } else {
	if (Pos->Parent_Index == BTREE_NODE_SIZE) {
            Prev_Index = Pos->Parent->Last;
	} else { 
            Prev_Index = BEN(Prev)(Pos->Parent, Pos->Parent_Index);
	}
	Retval = BEN(Get_Child)(Pos->Parent, Prev_Index);
    }

    return Retval;
}

static BEN(btree_node_t) *
BEN(Right_Node) (btree_t           *tree,
		 BEN(btree_node_t) *Pos)
{
    BEN(btree_node_t) *Retval;
    int Next_Index;

    if (Pos->Parent == NULL) {
	Retval = NULL;
    } else if (Pos->Parent_Index == BTREE_NODE_SIZE) {
	Retval = NULL;
    } else {
	if (Pos->Parent_Index == Pos->Parent->Last) {
            Retval = BEN(Get_Right_Child)(Pos->Parent);
	} else {
            Next_Index = BEN(Next)(Pos->Parent, Pos->Parent_Index);
            Retval = BEN(Get_Child)(Pos->Parent, Next_Index);
	}
    }

    return Retval;
}

static void
BEN(Insert_Shift_Left) (btree_t           *tree,
			BEN(btree_node_t) **Pos,
			int               *Index,
			btree_val_t       Val,
			BEN(btree_node_t) *Child,
			int               Rightmost)
{
    BEN(btree_node_t) *Search_Node;
    BEN(btree_node_t) *Curr_Node = *Pos;
    btree_val_t Hold_Val;
    BEN(btree_node_t) *Hold_Child = NULL;
    btree_val_t Tmp_Val;
    BEN(btree_node_t) *Tmp_Child, *Tmp_Child2;
    int Curr_Index;
    int Next_Index;
    int Parent_Index;

    if (Rightmost) {
         Hold_Val = Curr_Node->Vals[(int) Curr_Node->First];
         Curr_Node->Vals[(int) Curr_Node->First] = Val;
         if (Child != NULL) {
	     Hold_Child = BEN(Get_Child)(Curr_Node, Curr_Node->First);
	     BEN(Set_Child)(Curr_Node, Curr_Node->First, Child);
         }
         Curr_Node->First = BEN(Next)(Curr_Node, Curr_Node->First);
         Curr_Node->Last = BEN(Next)(Curr_Node, Curr_Node->Last);
         *Index = Curr_Node->Last;
         Search_Node = BEN(Left_Node)(tree, Curr_Node);
    } else if (*Index == (*Pos)->First) {
	Hold_Val = Val;
	Hold_Child = Child;
	
	Search_Node = BEN(Left_Node)(tree, Curr_Node);
	*Pos = Search_Node;
	*Index = BEN(Next)(Search_Node, Search_Node->Last);
    } else {
	Hold_Val = Curr_Node->Vals[(int) Curr_Node->First];
	Curr_Index = Curr_Node->First;
	Next_Index = BEN(Next)(Curr_Node, Curr_Index);
	while (Next_Index != *Index) {
	    Curr_Node->Vals[Curr_Index] = Curr_Node->Vals[Next_Index];
	    Curr_Index = Next_Index;
	    Next_Index = BEN(Next)(Curr_Node, Next_Index);
	}
	Curr_Node->Vals[Curr_Index] = Val;
	
	if (Child != NULL) {
	    Hold_Child = BEN(Get_Child)(Curr_Node, Curr_Node->First);
	    Curr_Index = Curr_Node->First;
	    Next_Index = BEN(Next)(Curr_Node, Curr_Index);
	    while (Next_Index != *Index) {
		Tmp_Child = BEN(Get_Child)(Curr_Node, Next_Index);
		BEN(Set_Child)(Curr_Node, Curr_Index, Tmp_Child);
		Curr_Index = Next_Index;
		Next_Index = BEN(Next)(Curr_Node, Next_Index);
	    }
	    BEN(Set_Child)(Curr_Node, Curr_Index, Child);
	}
	*Index = Curr_Index;
	Search_Node = BEN(Left_Node)(tree, Curr_Node);
    }

    while (BEN(Next)(Search_Node, Search_Node->Last)
	   == Search_Node->First)
    {
	if (Curr_Node->Parent_Index == BTREE_NODE_SIZE) {
            Parent_Index = Curr_Node->Parent->Last;
	} else {
            Parent_Index = BEN(Prev)(Curr_Node->Parent,
				     Curr_Node->Parent_Index);
	}

	Tmp_Val = Curr_Node->Parent->Vals[Parent_Index];
	Curr_Node->Parent->Vals[Parent_Index] = Hold_Val;
	Hold_Val = Search_Node->Vals[(int) Search_Node->First];
	Search_Node->Vals[(int) Search_Node->First] = Tmp_Val;

	if (Child != NULL) {
            Tmp_Child = BEN(Get_Child)(Search_Node, Search_Node->First);
            Tmp_Child2 = BEN(Get_Right_Child)(Search_Node);
            BEN(Set_Child)(Search_Node, Search_Node->First, Tmp_Child2);
	    BEN(Set_Right_Child)(Search_Node, Hold_Child);
            Hold_Child = Tmp_Child;
	}

	Search_Node->First = BEN(Next)(Search_Node, Search_Node->First);
	Search_Node->Last = BEN(Next)(Search_Node, Search_Node->Last);

	Curr_Node = Search_Node;
	Search_Node = BEN(Left_Node)(tree, Search_Node);
    }

    while (Curr_Node->Parent_Index == Curr_Node->Parent->First) {
         Curr_Node = Curr_Node->Parent;
    }

      if (Curr_Node->Parent_Index == BTREE_NODE_SIZE) {
         Parent_Index = Curr_Node->Parent->Last;
      } else {
         Parent_Index = BEN(Prev)(Curr_Node->Parent, Curr_Node->Parent_Index);
      }

      Tmp_Val = Curr_Node->Parent->Vals[Parent_Index];
      Curr_Node->Parent->Vals[Parent_Index] = Hold_Val;

      Search_Node->Last = BEN(Next)(Search_Node, Search_Node->Last);
      Search_Node->Vals[(int) Search_Node->Last] = Tmp_Val;
      if (Child != NULL) {
	  Tmp_Child = BEN(Get_Right_Child)(Search_Node);
	  BEN(Set_Child)(Search_Node, Search_Node->Last, Tmp_Child);
	  BEN(Set_Right_Child)(Search_Node, Hold_Child);
      }
}


static void
BEN(Insert_Shift_Right) (btree_t           *tree,
			 BEN(btree_node_t) **Pos,
			 int               *Index,
			 int               Rightmost,
			 btree_val_t       Val,
			 BEN(btree_node_t) *Child)
{
    BEN(btree_node_t) *Search_Node;
    BEN(btree_node_t) *Curr_Node = *Pos;
    btree_val_t Hold_Val;
    BEN(btree_node_t) *Hold_Child = NULL;
    btree_val_t Tmp_Val;
    BEN(btree_node_t) *Tmp_Child, *Tmp_Child2;
    int Curr_Index;
    int Prev_Index;

    if (Rightmost) {
	Hold_Val = Val;
	if (Child != NULL) {
            Hold_Child = BEN(Get_Right_Child)(Curr_Node);
	    BEN(Set_Right_Child)(Curr_Node, Child);
	}
	Search_Node = BEN(Right_Node)(tree, Curr_Node);
	*Pos = Search_Node;
	*Index = BEN(Prev)(Search_Node, Search_Node->First);
    } else {
	Curr_Index = Curr_Node->Last;
	Hold_Val = Curr_Node->Vals[Curr_Index];
	while (Curr_Index != *Index) {
            Prev_Index = BEN(Prev)(Curr_Node, Curr_Index);
            Curr_Node->Vals[Curr_Index] = Curr_Node->Vals[Prev_Index];
            Curr_Index = Prev_Index;
	}
	Curr_Node->Vals[*Index] = Val;

	if (Child != NULL) {
	    Hold_Child = BEN(Get_Right_Child)(Curr_Node);
	    Curr_Index = Curr_Node->Last;
	    Tmp_Child = BEN(Get_Child)(Curr_Node, Curr_Index);
	    BEN(Set_Right_Child)(Curr_Node, Tmp_Child);
	    while (Curr_Index != *Index) {
		Prev_Index = BEN(Prev)(Curr_Node, Curr_Index);
		Tmp_Child2 = BEN(Get_Child)(Curr_Node, Prev_Index);
		BEN(Set_Child)(Curr_Node, Curr_Index, Tmp_Child2);
		Curr_Index = Prev_Index;
	    }
	    BEN(Set_Child)(Curr_Node, *Index, Child);
	}
	Search_Node = BEN(Right_Node)(tree, Curr_Node);
    }

    while (BEN(Next)(Search_Node, Search_Node->Last) == Search_Node->First)
    {
	Tmp_Val = Curr_Node->Parent->Vals[(int) Curr_Node->Parent_Index];
	Curr_Node->Parent->Vals[(int) Curr_Node->Parent_Index] = Hold_Val;
	Hold_Val = Search_Node->Vals[(int) Search_Node->Last];
	Search_Node->Vals[(int) Search_Node->Last] = Tmp_Val;

	if (Child != NULL) {
            Tmp_Child = BEN(Get_Right_Child)(Search_Node);
	    Tmp_Child2 = BEN(Get_Child)(Search_Node, Search_Node->Last);
	    BEN(Set_Right_Child)(Search_Node, Tmp_Child2);
	    BEN(Set_Child)(Search_Node, Search_Node->Last, Hold_Child);
            Hold_Child = Tmp_Child;
	}

	Search_Node->First = BEN(Prev)(Search_Node, Search_Node->First);
	Search_Node->Last = BEN(Prev)(Search_Node, Search_Node->Last);

	Curr_Node = Search_Node;
	Search_Node = BEN(Right_Node)(tree, Search_Node);
    }

    while (Curr_Node->Parent_Index == BTREE_NODE_SIZE) {
	Curr_Node = Curr_Node->Parent;
    }
    Tmp_Val = Curr_Node->Parent->Vals[(int) Curr_Node->Parent_Index];
    Curr_Node->Parent->Vals[(int) Curr_Node->Parent_Index] = Hold_Val;
    Hold_Val = Tmp_Val;

    Search_Node->First = BEN(Prev)(Search_Node, Search_Node->First);
    Search_Node->Vals[(int) Search_Node->First] = Hold_Val;
    if (Child != NULL) {
	BEN(Set_Child)(Search_Node, Search_Node->First, Hold_Child);
    }
}

static BEN(btree_node_t) *
BEN(Alloc_Leaf) (btree_t *tree)
{
    BEN(btree_leaf_t) *New_Node;

    New_Node = malloc(sizeof(*New_Node));
    if (New_Node) {
	memset(New_Node, 0, sizeof(*New_Node));
	New_Node->node.Leaf = 1;
    }
    return &New_Node->node;
}

static BEN(btree_node_t) *
BEN(Alloc_Node) (btree_t *tree)
{
    BEN(btree_nleaf_t) *New_Node;

    New_Node = malloc(sizeof(*New_Node));
    if (New_Node)
	memset(New_Node, 0, sizeof(*New_Node));
    return &New_Node->node;
}

static int
BEN(Split_Node) (btree_t           *tree,
		 BEN(btree_node_t) **Pos,
		 int               *Index,
		 int               Rightmost,
		 btree_val_t       Val,
		 BEN(btree_node_t) *Child,
		 btree_val_t       *Parent_Val,
		 BEN(btree_node_t) **Parent_Child)
{
    BEN(btree_node_t) *New_Node, *Tmp_Node;
    int J;
    int Curr_Index;
    int Prev_Index;
    int I;

    if ((*Pos)->Leaf) {
	New_Node = BEN(Alloc_Leaf)(tree);
	if (New_Node == NULL) {
	    return BTREE_OUT_OF_MEMORY;
	}
    } else {
	New_Node = BEN(Alloc_Node)(tree);
	if (New_Node == NULL) {
	    return BTREE_OUT_OF_MEMORY;
	}
    }

    if (Rightmost || (BEN(Node_Item_Pos)(*Pos, *Index) > (BTREE_NODE_SIZE/2)))
    {
         Curr_Index = (*Pos)->First;
	 for (I=0; I<BTREE_NODE_SIZE/2; I++) {
	     New_Node->Vals[I] = (*Pos)->Vals[Curr_Index];
	     if (Child != NULL) {
		 Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
		 BEN(Set_Child)(New_Node, I, Tmp_Node);
	     }
	     Curr_Index = BEN(Next)(*Pos, Curr_Index);
         }

         if (Child != NULL) {
	     Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
	     BEN(Set_Right_Child)(New_Node, Tmp_Node);
         }
         *Parent_Val = (*Pos)->Vals[Curr_Index];
         Prev_Index = Curr_Index;
         Curr_Index = BEN(Next)(*Pos, Curr_Index);

         New_Node->First = 0;
         New_Node->Last = (BTREE_NODE_SIZE/2) - 1;
         if (Rightmost) {
	     (*Pos)->First = Curr_Index;

	     (*Pos)->Last = BEN(Next)(*Pos, (*Pos)->Last);
	     (*Pos)->Vals[(int) (*Pos)->Last] = Val;
	     if (Child != NULL) {
		 BEN(Set_Child)(*Pos, (*Pos)->Last, Child);
	     }
	     
	     *Index = (*Pos)->Last;
         } else {
	     (*Pos)->First = Curr_Index;
	     (*Pos)->Last = BEN(Next)(*Pos, (*Pos)->Last);
	     Curr_Index = (*Pos)->Last;
	     while (Curr_Index != *Index) {
		 Prev_Index = BEN(Prev)(*Pos, Curr_Index);
		 (*Pos)->Vals[Curr_Index] = (*Pos)->Vals[Prev_Index];
		 if (Child != NULL) {
		     Tmp_Node = BEN(Get_Child)(*Pos, Prev_Index);
		     BEN(Set_Child)(*Pos, Curr_Index, Tmp_Node);
		 }
		 Curr_Index = Prev_Index;
	     }
	     (*Pos)->Vals[*Index] = Val;
	     if (Child != NULL) {
		 BEN(Set_Child)(*Pos, *Index, Child);
	     }
         }

    } else if (BEN(Node_Item_Pos)(*Pos, *Index) == (BTREE_NODE_SIZE/2))
    {
	*Parent_Val = Val;

	Curr_Index = (*Pos)->First;
	for (I=0; I<BTREE_NODE_SIZE/2; I++) {
            New_Node->Vals[I] = (*Pos)->Vals[Curr_Index];
            if (Child != NULL) {
		Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
		BEN(Set_Child)(New_Node, I, Tmp_Node);
            }
            Curr_Index = BEN(Next)(*Pos, Curr_Index);
	}
	if (Child != NULL) {
	    BEN(Set_Right_Child)(New_Node, Child);
	}
	New_Node->First = 0;
	New_Node->Last = (BTREE_NODE_SIZE/2) - 1;

	(*Pos)->First = Curr_Index;

	*Pos = NULL;
    } else {
	Curr_Index = (*Pos)->First;
	J = 0;
	while (Curr_Index != *Index) {
            New_Node->Vals[J] = (*Pos)->Vals[Curr_Index];
            if (Child != NULL) {
		Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
		BEN(Set_Child)(New_Node, J, Tmp_Node);
            }
            Curr_Index = BEN(Next)(*Pos, Curr_Index);
            J = J + 1;
	}
	New_Node->Vals[J] = Val;
	*Index = J;
	if (Child != NULL) {
	    BEN(Set_Child)(New_Node, J, Child);
	}
	J = J + 1;
	while (J < (BTREE_NODE_SIZE/2)) {
            New_Node->Vals[J] = (*Pos)->Vals[Curr_Index];
            if (Child != NULL) {
		Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
		BEN(Set_Child)(New_Node, J, Tmp_Node);
            }
            Curr_Index = BEN(Next)(*Pos, Curr_Index);
            J = J + 1;
	}

	if (Child != NULL) {
	    Tmp_Node = BEN(Get_Child)(*Pos, Curr_Index);
	    BEN(Set_Right_Child)(New_Node, Tmp_Node);
	}
	New_Node->First = 0;
	New_Node->Last = (BTREE_NODE_SIZE/2) - 1;
	
	*Parent_Val = (*Pos)->Vals[Curr_Index];

	Curr_Index = BEN(Next)(*Pos, Curr_Index);
	(*Pos)->First = Curr_Index;

	*Pos = New_Node;
    }

    *Parent_Child = New_Node;

    return 0;
}

static int
BEN(Insert_Into_Node) (btree_t           *tree,
		       btree_val_t       Val,
		       BEN(btree_node_t) **Pos,
		       int               *Index,
		       BEN(btree_node_t) *Child,
		       int               Rightmost)
{
    BEN(btree_node_t) *Left_Search_Node;
    BEN(btree_node_t) *Right_Search_Node;
    int Done = 0;
    btree_val_t Parent_Val = Val;
    BEN(btree_node_t) *Parent_Child = Child;
    BEN(btree_node_t) *Parent;
    int Parent_Index;
    BEN(btree_node_t) *Curr_Node = *Pos, *Tmp_Node;
    int Work_Index = *Index;
    int Curr_Index;
    int Prev_Index;
    int Next_Index;
    int Local_Rightmost = Rightmost;
    int err;

    *Pos = NULL;
    while (! Done) {
	Next_Index = BEN(Next)(Curr_Node, Curr_Node->Last);
	if (Next_Index != Curr_Node->First) {
            if (Local_Rightmost) {
		Curr_Node->Vals[Next_Index] = Parent_Val;
		if (! Curr_Node->Leaf) {
		    BEN(Set_Child)(Curr_Node, Next_Index, Parent_Child);
		}
		if (*Pos == NULL) {
		    *Pos = Curr_Node;
		    *Index = Next_Index;
		}
            } else {
		Curr_Index = Next_Index;
		Prev_Index = Curr_Node->Last;
		while (Curr_Index != Work_Index) {
		    Curr_Node->Vals[Curr_Index] = Curr_Node->Vals[Prev_Index];
		    if (! Curr_Node->Leaf) {
			Tmp_Node = BEN(Get_Child)(Curr_Node, Prev_Index);
			BEN(Set_Child)(Curr_Node, Curr_Index, Tmp_Node);
		    }
		    Curr_Index = Prev_Index;
		    Prev_Index = BEN(Prev)(Curr_Node, Prev_Index);
		}
		Curr_Node->Vals[Work_Index] = Parent_Val;
		if (! Curr_Node->Leaf) {
		    BEN(Set_Child)(Curr_Node, Work_Index, Parent_Child);
		}
		*Index = Work_Index;
            }
            *Pos = Curr_Node;
            Curr_Node->Last = Next_Index;
            Done = 1;
	} else {
            Left_Search_Node = BEN(Left_Node)(tree, Curr_Node);
            Right_Search_Node = BEN(Right_Node)(tree, Curr_Node);
            while ((Left_Search_Node != NULL)
                   || (Right_Search_Node != NULL))
            {
		if (Left_Search_Node != NULL) {
		    if (BEN(Next)(Left_Search_Node, Left_Search_Node->Last)
			!= Left_Search_Node->First)
		    {
			BEN(Insert_Shift_Left)(tree,
					       &Curr_Node,
					       &Work_Index,
					       Parent_Val,
					       Parent_Child,
					       Local_Rightmost);
			if (*Pos == NULL) {
			    *Pos = Curr_Node;
			    *Index = Work_Index;
			}

			Done = 1;
			break;
		    }
		    Left_Search_Node = BEN(Left_Node)(tree, Left_Search_Node);
		}

		if (Right_Search_Node != NULL) {
		    if (BEN(Next)(Right_Search_Node, Right_Search_Node->Last)
			!= Right_Search_Node->First)
		    {
			BEN(Insert_Shift_Right)(tree,
						&Curr_Node,
						&Work_Index,
						Local_Rightmost,
						Parent_Val,
						Parent_Child);
			if (*Pos == NULL) {
			    *Pos = Curr_Node;
			    *Index = Work_Index;
			}

			Done = 1;
			break;
		    }
		    Right_Search_Node = BEN(Right_Node)(tree,
							Right_Search_Node);
		}
            }

            if (! Done) {
		if (Curr_Node == tree->Root) {
		    Parent = BEN(Alloc_Node)(tree);
		    if (Parent == NULL) {
			return BTREE_OUT_OF_MEMORY;
		    }
		    Parent->Parent = NULL;

		    BEN(Set_Right_Child)(Parent, Curr_Node);

		    err = BEN(Split_Node)(tree,
					  &Curr_Node,
					  &Work_Index,
					  Local_Rightmost,
					  Parent_Val,
					  Parent_Child,
					  &Parent_Val,
					  &Parent_Child);
		    if (err) {
			free(Parent);
			Curr_Node->Parent = NULL;
			return err;
		    }

		    if (*Pos == NULL) {
			if (Curr_Node == NULL) {
			    *Pos = Parent;
			    *Index = 0;
			} else {
			    *Pos = Curr_Node;
			    *Index = Work_Index;
			}
		    }
		    Parent->Vals[0] = Parent_Val;
		    BEN(Set_Child)(Parent, 0, Parent_Child);
		    tree->Root = Parent;
		    Parent->First = 0;
		    Parent->Last = 0;

		    Done = 1;
		} else {
		    Parent = Curr_Node->Parent;
		    Parent_Index = Curr_Node->Parent_Index;
		    err = BEN(Split_Node)(tree,
					  &Curr_Node,
					  &Work_Index,
					  Local_Rightmost,
					  Parent_Val,
					  Parent_Child,
					  &Parent_Val,
					  &Parent_Child);
		    if (err) {
			return err;
		    }

		    if ((Curr_Node != NULL) && (*Pos == NULL)) {
			*Pos = Curr_Node;
			*Index = Work_Index;
		    }
		    Curr_Node = Parent;

		    if (Parent_Index == BTREE_NODE_SIZE) {
			Local_Rightmost = 1;
		    } else {
			Work_Index = Parent_Index;
			Local_Rightmost = 0;
		    }
		}
            }
	}
    }

    return 0;
}

static int
BEN(Local_Add) (btree_t           *tree,
		btree_val_t       Val,
		BEN(btree_node_t) **Added_Pos,
		int               *Added_Index)
{
    BEN(btree_node_t) *Pos;
    int Index;
    int cmp_res;
    int err;

    Pos = tree->Root;
    if (tree->Count == 0) {
	Pos->First = 0;
	Pos->Last = 0;
	Pos->Vals[0] = Val;
	Index = 0;
    } else {
         Index = Pos->First;
	 for (;;) {
	     cmp_res = btree_cmp_key(Val, Pos->Vals[Index]);
	     if (cmp_res <= 0) {
		 if ((! tree->Allow_Duplicates)
		     && (cmp_res == 0))
		 {
		     return BTREE_ITEM_ALREADY_EXISTS;
		 }
		 if (Pos->Leaf) {
		     err = BEN(Insert_Into_Node)(tree,
						 Val,
						 &Pos,
						 &Index,
						 NULL,
						 0);
		     if (err) {
			 return err;
		     }
		     break;
		 } else {
		     Pos = BEN(Get_Child)(Pos, Index);
		     Index = Pos->First;
		 }
	     } else if (Index == Pos->Last) {
		 if (Pos->Leaf) {
		     err = BEN(Insert_Into_Node)(tree,
						 Val,
						 &Pos,
						 &Index,
						 NULL,
						 1);
		     if (err) {
			 return err;
		     }
		     break;
		 } else {
		     Pos = BEN(Get_Right_Child)(Pos);
		     Index = Pos->First;
		 }
	     } else {
		 Index = BEN(Next)(Pos, Index);
	     }
         }
    }
    
    tree->Count = tree->Count + 1;
    tree->Update = tree->Update + 1;

    *Added_Pos = Pos;
    *Added_Index = Index;

    return 0;
}

#if BTREE_NEEDS & BTREE_NEEDS_DELETE
static void
BEN(Delete_From_Node) (btree_t           *tree,
		       BEN(btree_node_t) *Pos,
		       int               Index)
{
    BEN(btree_node_t) *Tmp_Node;
    int Curr_Index;
    int Prev_Index;

    Curr_Index = Index;
    Prev_Index = BEN(Prev)(Pos, Index);
    while (Curr_Index != Pos->First) {
	Pos->Vals[Curr_Index] = Pos->Vals[Prev_Index];
	if (! Pos->Leaf) {
	    Tmp_Node = BEN(Get_Child)(Pos, Prev_Index);
	    BEN(Set_Child)(Pos, Curr_Index, Tmp_Node);
	}
	Curr_Index = Prev_Index;
	Prev_Index = BEN(Prev)(Pos, Prev_Index);
    }
    Pos->First = BEN(Next)(Pos, Pos->First);
}

static void
BEN(Delete_Shift_Left) (btree_t           *tree,
			BEN(btree_node_t) *Pos,
			BEN(btree_node_t) **Next_Pos,
			int               *Next_Index)
{
    BEN(btree_node_t) *Search_Node, *Tmp_Node;
    BEN(btree_node_t) *Curr_Node = Pos;
    BEN(btree_node_t) *Parent_Node;

    for (;;) {
	Search_Node = BEN(Right_Node)(tree, Curr_Node);

	Parent_Node = Curr_Node;
	while (Parent_Node->Parent_Index == BTREE_NODE_SIZE) {
            Parent_Node = Parent_Node->Parent;
	}

	Curr_Node->Last = BEN(Next)(Curr_Node, Curr_Node->Last);
	if ((Parent_Node->Parent == *Next_Pos)
	    && (Parent_Node->Parent_Index == *Next_Index))
	{
            *Next_Pos = Curr_Node;
            *Next_Index = Curr_Node->Last;
	}
	if ((Search_Node == *Next_Pos)
	    && (Search_Node->First == *Next_Index))
	{
            *Next_Pos = Parent_Node->Parent;
            *Next_Index = Parent_Node->Parent_Index;
	}
	Curr_Node->Vals[(int) Curr_Node->Last]
	    = Parent_Node->Parent->Vals[(int) Parent_Node->Parent_Index];
	Parent_Node->Parent->Vals[(int) Parent_Node->Parent_Index]
	    = Search_Node->Vals[(int) Search_Node->First];
	if (! Curr_Node->Leaf) {
	    Tmp_Node = BEN(Get_Right_Child)(Curr_Node);
            BEN(Set_Child)(Curr_Node, Curr_Node->Last, Tmp_Node);
            Tmp_Node = BEN(Get_Child)(Search_Node, Search_Node->First);
            BEN(Set_Right_Child)(Curr_Node, Tmp_Node);
	}
	Search_Node->First = BEN(Next)(Search_Node, Search_Node->First);

	if (BEN(Node_Count)(Search_Node) >= (BTREE_NODE_SIZE / 2))
	{
	    break;
	}
	Curr_Node = Search_Node;
    }
}

static void
BEN(Delete_Shift_Right) (btree_t           *tree,
			 BEN(btree_node_t) *Pos,
			 BEN(btree_node_t) **Next_Pos,
			 int               *Next_Index)
{
    BEN(btree_node_t) *Search_Node, *Tmp_Node;
    BEN(btree_node_t) *Curr_Node = Pos;
    BEN(btree_node_t) *Parent_Node;
    int Parent_Index;

    for (;;) {
	Search_Node = BEN(Left_Node)(tree, Curr_Node);

	Parent_Node = Curr_Node;
	while (Parent_Node->Parent_Index == Parent_Node->Parent->First) {
            Parent_Node = Parent_Node->Parent;
	}

	if (Parent_Node->Parent_Index == BTREE_NODE_SIZE) {
            Parent_Index = Parent_Node->Parent->Last;
	} else {
            Parent_Index = BEN(Prev)(Parent_Node->Parent,
				     Parent_Node->Parent_Index);
	}

	Curr_Node->First = BEN(Prev)(Curr_Node, Curr_Node->First);
	if ((Parent_Node->Parent == *Next_Pos)
	    && (Parent_Index == *Next_Index))
	{
            *Next_Pos = Curr_Node;
            *Next_Index = Curr_Node->First;
	}
	if ((Search_Node == *Next_Pos)
	    && (Search_Node->Last == *Next_Index))
	{
            *Next_Pos = Parent_Node->Parent;
            *Next_Index = Parent_Index;
	}
	Curr_Node->Vals[(int) Curr_Node->First]
	    = Parent_Node->Parent->Vals[Parent_Index];
	Parent_Node->Parent->Vals[Parent_Index]
	    = Search_Node->Vals[(int) Search_Node->Last];
	if (! Curr_Node->Leaf) {
	    Tmp_Node = BEN(Get_Right_Child)(Search_Node);
            BEN(Set_Child)(Curr_Node, Curr_Node->First, Tmp_Node);

	    Tmp_Node = BEN(Get_Child)(Search_Node, Search_Node->Last);
            BEN(Set_Right_Child)(Search_Node, Tmp_Node);
	}
	Search_Node->Last = BEN(Prev)(Search_Node, Search_Node->Last);

	if (BEN(Node_Count)(Search_Node) >= (BTREE_NODE_SIZE / 2))
	{
	    break;
	}
	Curr_Node = Search_Node;
    }
}

static void
BEN(Combine_Nodes) (btree_t           *tree,
		    BEN(btree_node_t) *Node1,
		    BEN(btree_node_t) *Node2,
		    BEN(btree_node_t) **Next_Pos,
		    int               *Next_Index)
{
    BEN(btree_node_t) *Tmp_Node;

    Node2->First = BEN(Prev)(Node2, Node2->First);
    if ((Node1->Parent == *Next_Pos)
	&& (Node1->Parent_Index == *Next_Index))
    {
	*Next_Pos = Node2;
	*Next_Index = Node2->First;
    }
    Node2->Vals[(int) Node2->First]
	= Node1->Parent->Vals[(int) Node1->Parent_Index];
    if (! Node1->Leaf) {
	Tmp_Node = BEN(Get_Right_Child)(Node1);
	BEN(Set_Child)(Node2, Node2->First, Tmp_Node);
    }

    for (;;) {
	Node2->First = BEN(Prev)(Node2, Node2->First);

	if ((Node1 == *Next_Pos)
	    && (Node1->Last == *Next_Index))
	{
            *Next_Pos = Node2;
            *Next_Index = Node2->First;
	}
	Node2->Vals[(int) Node2->First] = Node1->Vals[(int) Node1->Last];
	if (! Node1->Leaf) {
	    Tmp_Node = BEN(Get_Child)(Node1, Node1->Last);
            BEN(Set_Child)(Node2, Node2->First, Tmp_Node);
	}

	if (Node1->Last == Node1->First) {
	    break;
	}

	Node1->Last = BEN(Prev)(Node1, Node1->Last);
    }
}

static void
BEN(Local_Delete) (btree_t           *tree,
		   BEN(btree_node_t) *Pos,
		   int               Index,
		   BEN(btree_node_t) **New_Next_Pos,
		   int               *New_Next_Index,
		   int               *Is_End)
{
    BEN(btree_node_t) *Node = Pos;
    int Curr_Index = Index;
    int Done = 0;
    BEN(btree_node_t) *Left_Search_Node;
    BEN(btree_node_t) *Right_Search_Node;
    BEN(btree_node_t) *Combine_Left_Node;
    BEN(btree_node_t) *Combine_Right_Node;
    int Next_Index;
    int Prev_Index;
    BEN(btree_node_t) *Return_Next_Pos = Pos;
    int Return_Next_Index = Index;
    int Local_Is_End;

    if (! Node->Leaf) {
	Node = BEN(Get_Child)(Node, Curr_Index);
	while (! Node->Leaf) {
            Node = BEN(Get_Right_Child)(Node);
	}
	Curr_Index = Node->Last;
	Pos->Vals[Index] = Node->Vals[Curr_Index];
    }

    BEN(Local_Next)(tree, &Return_Next_Pos, &Return_Next_Index, &Local_Is_End);
    *Is_End = Local_Is_End;
    if (Local_Is_End) {
	Return_Next_Pos = NULL;
    }

    while (! Done) {
	if (BEN(Node_Count)(Node) > (BTREE_NODE_SIZE / 2)) {
            BEN(Delete_From_Node)(tree, Node, Curr_Index);
            Done = 1;
	} else {
            Left_Search_Node = BEN(Left_Node)(tree, Node);
            Right_Search_Node = BEN(Right_Node)(tree, Node);
            while ((Left_Search_Node != NULL)
                   || (Right_Search_Node != NULL))
            {
		if (Left_Search_Node != NULL) {
		    if (BEN(Node_Count)(Left_Search_Node)
			> (BTREE_NODE_SIZE / 2))
		    {
			BEN(Delete_From_Node)(tree, Node, Curr_Index);

			BEN(Delete_Shift_Right)(tree,
						Node,
						&Return_Next_Pos,
						&Return_Next_Index);

			Done = 1;
			break;
		    }
		    Left_Search_Node = BEN(Left_Node)(tree, Left_Search_Node);
		}

		if (Right_Search_Node != NULL) {
		    if (BEN(Node_Count)(Right_Search_Node)
			> (BTREE_NODE_SIZE / 2))
		    {
			BEN(Delete_From_Node)(tree, Node, Curr_Index);

			BEN(Delete_Shift_Left)(tree,
					       Node,
					       &Return_Next_Pos,
					       &Return_Next_Index);

			Done = 1;
			break;
		    }
		    Right_Search_Node= BEN(Right_Node)(tree, Right_Search_Node);
		}
            }
	}

	if (! Done) {
            if (Node == tree->Root) {
		if (BEN(Node_Count)(Node) == 1) {
		    if (! Node->Leaf) {
			tree->Root = BEN(Get_Right_Child)(Node);
			tree->Root->Parent = NULL;
			tree->Root->Parent_Index = 0;
			free(Node);
		    }
		} else {
		    BEN(Delete_From_Node)(tree, Node, Curr_Index);
		}
		Done = 1;
            } else {
		if (Node->Parent_Index == Node->Parent->First) {
		    Combine_Left_Node = Node;
		    Next_Index = BEN(Next)(Node->Parent, Node->Parent_Index);
		    if (Node->Parent_Index == Node->Parent->Last) {
			Combine_Right_Node
			    = BEN(Get_Right_Child)(Node->Parent);
		    } else {
			Combine_Right_Node
			    = BEN(Get_Child)(Node->Parent, Next_Index);
		    }
		} else {
		    Combine_Right_Node = Node;
		    if (Node->Parent_Index == BTREE_NODE_SIZE) {
			Prev_Index = Node->Parent->Last;
		    } else {
			Prev_Index
			    = BEN(Prev)(Node->Parent, Node->Parent_Index);
		    }
		    Combine_Left_Node
			= BEN(Get_Child)(Node->Parent, Prev_Index);
		}

		BEN(Delete_From_Node)(tree, Node, Curr_Index);

		Curr_Index = Combine_Left_Node->Parent_Index;
		Node = Node->Parent;
		BEN(Combine_Nodes)(tree,
				   Combine_Left_Node,
				   Combine_Right_Node,
				   &Return_Next_Pos,
				   &Return_Next_Index);
		free(Combine_Left_Node);
            }
	}
    }

    *New_Next_Pos = Return_Next_Pos;
    *New_Next_Index = Return_Next_Index;

    tree->Count = tree->Count - 1;
    tree->Update = tree->Update + 1;
}
#endif

BTREE_NAMES_LOCAL int
BEN(add) (btree_t     *tree,
	  btree_val_t val)
{
    BEN(btree_node_t) *pos;
    int          index;

    return BEN(Local_Add)(tree, val, &pos, &index);
}

#if BTREE_NEEDS & BTREE_NEEDS_DELETE
BTREE_NAMES_LOCAL int
BEN(delete) (btree_t     *tree,
	     btree_val_t val,
	     btree_val_t *next,
	     int         *is_end)
{
    BEN(btree_node_t) *pos;
    int index;
    int found;
    BEN(btree_node_t) *next_pos;
    int next_index;
    int local_is_end;

    BEN(Local_Search)(tree, val, &pos, &index, &found);
    if (! found) {
	return BTREE_ITEM_NOT_FOUND;
    }

    BEN(Local_Delete)(tree,
		      pos,
		      index,
		      &next_pos,
		      &next_index,
		      &local_is_end);
    if (! local_is_end) {
	if (next != NULL) {
	    *next = next_pos->Vals[next_index];
	}
    }
    if (is_end != NULL) {
	*is_end = local_is_end;
    }

    return 0;
}
#endif

BTREE_NAMES_LOCAL int
BEN(search) (btree_t     *tree,
	     btree_val_t val,
	     btree_val_t *item,
	     int         closest_op)
{
    BEN(btree_node_t) *pos;
    int          index;
    int          found;
    int          rv = 0;
    
    BEN(Local_Search)(tree, val, &pos, &index, &found);
    if (! found) {
	if (closest_op == BTREE_NO_CLOSEST) {
	    return BTREE_ITEM_NOT_FOUND;
	} else if (pos == NULL) {
	    return BTREE_AT_END_OF_TREE;
	}
	if (closest_op == BTREE_CLOSEST_PREV) {
	    int is_end;

	    if (btree_cmp_key(val, pos->Vals[index]) < 0) {
		BEN(Local_Prev)(tree, &pos, &index, &is_end);
		if (is_end) {
		    return BTREE_AT_END_OF_TREE;
		}
	    }
	} else {
	    int is_end;

	    if (btree_cmp_key(val, pos->Vals[index]) > 0) {
		BEN(Local_Next)(tree, &pos, &index, &is_end);
		if (is_end) {
		    return BTREE_AT_END_OF_TREE;
		}
	    }
	}
    }
    if (item != NULL) {
	*item = pos->Vals[index];
    }

    return rv;
}

#if BTREE_NEEDS & BTREE_NEEDS_FIRST
BTREE_NAMES_LOCAL int
BEN(first) (btree_t     *tree,
	    btree_val_t *item)
{
    BEN(btree_node_t) *pos;
    int index;
    int is_end;
    
    BEN(Local_First)(tree, &pos, &index, &is_end);
    if (is_end) {
	return BTREE_ITEM_NOT_FOUND;
    }
    if (item != NULL) {
	*item = pos->Vals[index];
    }

    return 0;
}
#endif

#if BTREE_NEEDS & BTREE_NEEDS_LAST
BTREE_NAMES_LOCAL int
BEN(last) (btree_t     *tree,
	   btree_val_t *item)
{
    BEN(btree_node_t) *pos;
    int index;
    int is_end;
    
    BEN(Local_Last)(tree, &pos, &index, &is_end);
    if (is_end) {
	return BTREE_ITEM_NOT_FOUND;
    }
    if (item != NULL) {
	*item = pos->Vals[index];
    }

    return 0;
}
#endif

#if BTREE_NEEDS & BTREE_NEEDS_NEXT
BTREE_NAMES_LOCAL int
BEN(next) (btree_t     *tree,
	   btree_val_t val,
	   btree_val_t *next_item)
{
    BEN(btree_node_t) *pos;
    int index;
    int found;
    int is_end;
    
    BEN(Local_Search)(tree, val, &pos, &index, &found);
    if (! found) {
	return BTREE_ITEM_NOT_FOUND;
    }
    BEN(Local_Next)(tree, &pos, &index, &is_end);
    if (is_end) {
	return BTREE_AT_END_OF_TREE;
    }
    if (next_item != NULL) {
	*next_item = pos->Vals[index];
    }

    return 0;
}
#endif

#if BTREE_NEEDS & BTREE_NEEDS_PREV
BTREE_NAMES_LOCAL int
BEN(prev) (btree_t     *tree,
			 btree_val_t val,
			 btree_val_t *prev_item)
{
    BEN(btree_node_t) *pos;
    int index;
    int found;
    int is_end;
    
    BEN(Local_Search)(tree, val, &pos, &index, &found);
    if (! found) {
	return BTREE_ITEM_NOT_FOUND;
    }
    BEN(Local_Prev)(tree, &pos, &index, &is_end);
    if (is_end) {
	return BTREE_AT_END_OF_TREE;
    }
    if (prev_item != NULL) {
	*prev_item = pos->Vals[index];
    }

    return 0;
}
#endif

BTREE_NAMES_LOCAL int
BEN(init) (btree_t *tree)
{
    if (tree->Root)
	return BTREE_ITEM_ALREADY_EXISTS;
    tree->Root = BEN(Alloc_Leaf)(tree);
    if (!tree->Root)
	return BTREE_OUT_OF_MEMORY;
    tree->Root->Parent = NULL;
    tree->Count = 0;
    tree->Allow_Duplicates = 0;
    tree->Update = 0;
    return 0;
}

static void
BEN(free_node)(btree_t           *tree,
	       BEN(btree_node_t) *node)
{
    int i;
    BEN(btree_node_t) *Tmp_Node;

    if (! node->Leaf) {
	BEN(btree_nleaf_t) *nl = BEN(container_of_nleaf)(node);

	for (i=node->First; i != node->Last; i=BEN(Next)(node, i))
	{
	    Tmp_Node = BEN(Get_Child)(node, i);
	    BEN(free_node)(tree, Tmp_Node);
	}
	Tmp_Node = BEN(Get_Child)(node, i);
	BEN(free_node)(tree, Tmp_Node);
	Tmp_Node = BEN(Get_Right_Child)(node);
	BEN(free_node)(tree, Tmp_Node);
	free(nl);
    } else {
	BEN(btree_leaf_t) *l = BEN(container_of_leaf)(node);
	free(l);
    }
}

BTREE_NAMES_LOCAL void
BEN(free) (btree_t *tree)
{
    if (tree->Root == NULL)
	return;

    BEN(free_node)(tree, tree->Root);
    tree->Root = NULL;
    tree->Count = 0;
}
