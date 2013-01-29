// -*- c++ -*-
// Generated by gtkmmproc -- DO NOT MODIFY!
#ifndef _GOOCANVASMM_LINEDASH_H
#define _GOOCANVASMM_LINEDASH_H


#include <glibmm.h>

/* Copyright (C) 1998-2006 The gtkmm Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <glibmm/object.h>
#include <goocanvasutils.h>


#ifndef DOXYGEN_SHOULD_SKIP_THIS
extern "C" { typedef struct _GooCanvasLineDash GooCanvasLineDash; }
#endif

namespace Goocanvas
{

class LineDash
{
  public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  typedef LineDash CppObjectType;
  typedef GooCanvasLineDash BaseObjectType;

  static GType get_type() G_GNUC_CONST;
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

  LineDash();

  explicit LineDash(GooCanvasLineDash* gobject, bool make_a_copy = true);

  LineDash(const LineDash& other);
  LineDash& operator=(const LineDash& other);

  ~LineDash();

  void swap(LineDash& other);

  ///Provides access to the underlying C instance.
  GooCanvasLineDash*       gobj()       { return gobject_; }

  ///Provides access to the underlying C instance.
  const GooCanvasLineDash* gobj() const { return gobject_; }

  ///Provides access to the underlying C instance. The caller is responsible for freeing it. Use when directly setting fields in structs.
  GooCanvasLineDash* gobj_copy() const;

protected:
  GooCanvasLineDash* gobject_;

private:

  
public:

//TODO: Wrap
//GooCanvasLineDash* goo_canvas_line_dash_newv  (gint               num_dashes,
//                                               double            *dashes);
  

};

}


namespace Goocanvas
{

/** @relates Goocanvas::LineDash
 * @param lhs The left-hand side
 * @param rhs The right-hand side
 */
inline void swap(LineDash& lhs, LineDash& rhs)
  { lhs.swap(rhs); }

} // namespace Goocanvas

namespace Glib
{

/** A Glib::wrap() method for this object.
 * 
 * @param object The C instance.
 * @param take_copy False if the result should take ownership of the C instance. True if it should take a new copy or ref.
 * @result A C++ instance that wraps this C instance.
 *
 * @relates Goocanvas::LineDash
 */
Goocanvas::LineDash wrap(GooCanvasLineDash* object, bool take_copy = false);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
template <>
class Value<Goocanvas::LineDash> : public Glib::Value_Boxed<Goocanvas::LineDash>
{};
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

} // namespace Glib


#endif /* _GOOCANVASMM_LINEDASH_H */

