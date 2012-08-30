#!/usr/bin/env python

""" ====================================
    fileMethods.py - File Access Methods 
    ====================================
    
    Some routines for reading and writing plaintext config/data files, shared throughout the toolkit.
"""

import re, types

def readFromFile(fileName):
    """
    A simple method for reading in data from text files of the following format::

        ========= SECTION1 ==========

        # COMMENT

        HEADER1: # COMMENT
        DATA1
        DATA2

        HEADER2: # COMMENT
        DATA3

        ========= SECTION2 ===========

        HEADER1:
        DATA1
    
    Given a file with the above contents, the function will return the following dictionary of dictionaries::

        { 'SECTION1' : { 'HEADER1': ['DATA1', 'DATA2'],
                         'HEADER2': ['DATA3'] },
        { 'SECTION2' : { 'HEADER1': ['DATA1'] } }

    NOTE:
        * Headers *must* be followed by a colon
        * Section names must have at least one equals sign on both sides
        * Dictionary keys will always be normalized so that only the first letter is capitalized (*CURRENTLY DISABLED*)
        * All items are returned as strings and should be cast to the appropriate type before
          use, if appropriate
        * Lines beginning with ``#`` are treated as comments
        * Blank lines at the end of sections are ignored
        * Any data without a section title will be returned under the empty string key: ``''``
        * If no section titles are present, the outer dictionary will be ommitted
    """

    # FIXME: This file reading function is very fragile; we should catch exceptions
    try:
        f = open(fileName, "r")
    except IOError:
        print "ERROR: Cannot open file %s" % fileName
        return None

    # A regex to search for sections
    p_sec = re.compile(r"^=+\s*(?P<title>[^=]+)=+")

    # A regex to search for headers
    p_head = re.compile(r"^(?P<key>\w+):\s*(?:#.*)?$")

    title = ''
    key = None
    data = {'':{}}  # Initialize, so a section title is optional

    for line in f:
        if key != "Spec" and (line[0] == "#" or line.strip() == ""): continue # Ignore commented line

        m_sec = p_sec.match(line)
        m_head = p_head.match(line)

        if m_sec:
            # We've found a new section title; create a new entry in the dict
            title = m_sec.group('title').strip()
            data[title] = {}
            key = None # Reset key
        elif m_head:
            # We've found a new header; create a new entry in the subdict
            key = m_head.group('key')
            data[title][key] = []
        else:
            # We've found non-header data
            if key is None:
                # We never found a header for this data!  Unless it is a comment or a blank line, assume the file is bad.
                print "ERROR: Found data before header while reading from file %s." % (fileName)
                return None
            else:
                data[title][key].append(line.strip())

    f.close()

    if key is None:
        # We found no data; assume the file is bad
        print "ERROR: Found no data while reading from file %s." % (fileName)
        return None

    # Remove the blank key if everything was under section titles
    if len(data['']) == 0:
        del data['']

    # Go back through each section and remove any trailing blank entries
    for key1, vals1 in data.iteritems():
        for key2, vals2 in vals1.iteritems():
            while(len(vals2) > 0 and vals2[-1] is ''):
                del vals2[-1]
    
    # Break out of outer dictionary if we don't have any section titles
    if data.keys() == ['']:
        data = data['']

    return data

def writeToFile(fileName, data, comments={}):
    """
    A simple method for writing data to text files of the format described in the :meth:`readFromFile`
    function above.

    All data will be output in key-sorted order for the sake of consistency.

    Any items in the comments hash whose keys match those of the data hash will be included
    as comments in the appropriate section of the file.

    If it exists, the comment keyed as ``"FILE_HEADER"`` will be added to the top of the output file.
    """

    # If we don't have any section titles, make a placeholder second layer
    if type(data[data.keys()[0]]) is types.ListType:
        data = {'': data}

    f = open(fileName, "w")
    
    if "FILE_HEADER" in comments:
        for line in comments["FILE_HEADER"].splitlines():
            print >>f, "# " + line
        print >>f

    for title in sorted(data.keys()):
        section = data[title]

        if title is not '':
            # Print out the section header
            print >>f, "\n======== %s ========\n" % title

        for header in sorted(section.keys()):
            values = data[title][header]

            # Output a comment if we have one available
            if header in comments:
                print >>f, properCase(header) + ": # " + comments[header]
            else:
                print >>f, properCase(header) + ":"

            # Plain values are ok too!
            if type(values) is not types.ListType:
                values = [str(values)]

            for value in values:
                print >>f, value
            print >>f  # Put a blank line in between sections, just because it's prettier that way 
    f.close()

def properCase(str):
    """
    Returns a copy of a string, with the first letter capitalized and all others lower-case
    (*CURRENTLY DOES NOTHING*, and I'm not sure why that change was made)
    """

    return str
    #return str[0].upper() + str[1:].lower()

