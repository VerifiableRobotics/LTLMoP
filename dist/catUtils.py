import textwrap

def patrickSays(msg):
    # Wrap to an appropriate width between 10 and 60 characters
    # This is super inefficient... but we are using modern computers!
    for wrap_width in xrange(10,60):
        wrapped_msg = textwrap.wrap(msg, wrap_width)
        if len(wrapped_msg) <= 2:
            break
    
    # Fill to at least 2 lines
    while (len(wrapped_msg) < 2):
        wrapped_msg.append("")

    # This ASCII cat is by 'Hilary'; taken without permission from http://www.doghause.com/ascii.asp
    plain_cat = r"""
          /\___/\**
    /   /  .   . \*
    \   \    ^   /*
    \  /        \**
    \ /______  \***
*******************""".replace("*"," ").split("\n")
    
    plain_bubble = r"""
    __{0}_
   /  %-{1}s \
  <   %-{1}s  |
   |  %-{1}s  |
    \_{0}_/
    """.format("_"*wrap_width, wrap_width).split("\n")
    
    # Make the bubble the right size
    extra_height = len(wrapped_msg) - 2
    plain_bubble = plain_bubble[0:4] + [plain_bubble[4]]*max(0,extra_height) + plain_bubble[5:]
    if extra_height <= 0:
        plain_bubble[4] = plain_bubble[4].replace(r" \_", r"\__") # Tweaking

    cat_index, bubble_index, msg_index = (0,0,0)
    while bubble_index < len(plain_bubble):
        if "%" in plain_bubble[bubble_index]:
            print plain_cat[cat_index] + plain_bubble[bubble_index] % (wrapped_msg[msg_index])
            msg_index += 1
        else:
            print plain_cat[cat_index] + plain_bubble[bubble_index]

        cat_index = min(cat_index+1, len(plain_cat)-1)
        bubble_index += 1

if __name__ == "__main__":
    patrickSays("Hi.")
    patrickSays("Now we are engaged in a great civil war, testing whether that nation, or any nation so conceived and so dedicated, can long endure. We are met on a great battlefield of that war. We have come to dedicate a portion of that field, as a final resting place for those who here gave their lives that that nation might live. It is altogether fitting and proper that we should do this.")
    patrickSays("catUtils appears to be working!")
