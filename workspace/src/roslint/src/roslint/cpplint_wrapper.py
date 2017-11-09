# Software License Agreement (BSD License)
#
# copyright (c) 2014-2015 roslint contributors
# all rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from roslint import cpplint
from roslint.cpplint import Match, IsBlankLine, main
from functools import partial

import os.path
import re

# Line length as per the ROS C++ Style Guide
cpplint._line_length = 120


def patch(original_module):
    """ Decorator to easily allow wrapping/overriding of the Check* functions in cpplint. Should
        decorate a function which matches the signature of the function it replaces expect with
        the addition of a fn parameter, which is a pass-through of the replaced function, in case
        the replacement would like call through to the original functionality. """
    def wrap(override_fn):
        original_fn = getattr(original_module, override_fn.__name__)
        setattr(original_module, override_fn.__name__, partial(override_fn, original_fn))

        # Don't actually modify the function being decorated.
        return override_fn
    return wrap


def makeErrorFn(original_fn, suppress_categories, suppress_message_matches):
    """ Create a return a wrapped version of the error-report function which suppresses specific
        error categories. """
    def newError(filename, linenum, category, confidence, message):
        if category in suppress_categories:
            return
        if True in [bool(Match(r, message)) for r in suppress_message_matches]:
            return
        original_fn(filename, linenum, category, confidence, message)
    return newError


@patch(cpplint)
def GetHeaderGuardCPPVariable(fn, filename):
    """ Replacement for the function which determines the header guard variable, to pick one which
        matches ROS C++ Style. """
    var_parts = list()
    head = filename
    while head:
        head, tail = os.path.split(head)
        var_parts.insert(0, tail)
        if head.endswith('include') or tail == "":
            break
    return re.sub(r'[-./\s]', '_', "_".join(var_parts)).upper()


@patch(cpplint)
def CheckBraces(fn, filename, clean_lines, linenum, error):
    """ Complete replacement for cpplint.CheckBraces, since the brace rules for ROS C++ Style
        are completely different from the Google style guide ones. """
    line = clean_lines.elided[linenum]
    if Match(r'^(.*){(.*)}.?$', line):
        # Special case when both braces are on the same line together, as is the
        # case for one-line getters and setters, for example, or rows of a multi-
        # dimenstional array initializer.
        pass
    else:
        # Line does not contain both an opening and closing brace.
        m = Match(r'^(.*){(.*)$', line)
        if m and not (IsBlankLine(m.group(1))):
            # Line contains a starting brace and is not empty, uh oh.
            if "=" in line and Match(r'\)( *){$', line):
                # Opening brace is permissable in case of an initializer.
                pass
            else:
                error(filename, linenum, 'whitespace/braces', 4,
                      'when starting a new scope, { should be on a line by itself')
        m = Match(r'^(.*)}(.*)$', line)
        if m and (not IsBlankLine(m.group(1)) or not IsBlankLine(m.group(2))):
            if m.group(2) != ";":
                error(filename, linenum, 'whitespace/braces', 4,
                      '} should be on a line by itself')
    pass


@patch(cpplint)
def CheckIncludeLine(fn, filename, clean_lines, linenum, include_state, error):
    """ Run the function to get include state, but suppress all the errors, since
        ROS C++ Style is silent on include order, and contains no prohibition on use of streams. """
    fn(filename, clean_lines, linenum, include_state,
       makeErrorFn(error, ['build/include_order', 'build/include_alpha', 'readability/streams'], []))


@patch(cpplint)
def CheckSpacing(fn, filename, clean_lines, linenum, nesting_state, error):
    """ Do most of the original Spacing checks, but suppress the ones related to braces, since
        the ROS C++ Style rules are different. """
    fn(filename, clean_lines, linenum, nesting_state,
       makeErrorFn(error, ['readability/braces', 'whitespace/braces'], []))


@patch(cpplint)
def ProcessLine(fn, filename, file_extension, clean_lines, line,
                include_state, function_state, nesting_state, error,
                extra_check_functions=[]):
    """ Squelch the error about access control indents. """
    fn(filename, file_extension, clean_lines, line,
       include_state, function_state, nesting_state,
       makeErrorFn(error, [], [r'(.*)should be indented \+1 space inside(.*)']),
       extra_check_functions=[])


@patch(cpplint)
def CheckEmptyBlockBody(fn, filename, clean_lines, linenum, error):
    """ Look for empty loop/conditional body with only a single semicolon,
        but allow ros-style do while loops. """
    from cpplint import CloseExpression

    # Search for loop keywords at the beginning of the line.  Because only
    # whitespaces are allowed before the keywords, this will also ignore most
    # do-while-loops, since those lines should start with closing brace.
    #
    # We also check "if" blocks here, since an empty conditional block
    # is likely an error.
    line = clean_lines.elided[linenum]
    matched = Match(r'\s*(for|while|if)\s*\(', line)
    if matched:
        # Find the end of the conditional expression
        (end_line, end_linenum, end_pos) = CloseExpression(
            clean_lines, linenum, line.find('('))

        # Output warning if what follows the condition expression is a
        # semicolon.  No warning for all other cases, including
        # whitespace or newline, since we have a separate check for
        # semicolons preceded by whitespace.
        if end_pos >= 0 and Match(r';', end_line[end_pos:]):
            if matched.group(1) == 'if':
                error(filename, end_linenum,
                      'whitespace/empty_conditional_body', 5,
                      'Empty conditional bodies should use {}')
            elif matched.group(1) == 'while' and linenum is not 0 \
                    and "}" in clean_lines.elided[linenum-1]:
                # Don't report an error for ros style do-whiles. Works
                # by checking for a closing brace on the previous
                # line, since that means it's probably a do-while
                # loop.
                return
            else:
                error(filename, end_linenum, 'whitespace/empty_loop_body', 5,
                      'Empty loop bodies should use {} or continue')
