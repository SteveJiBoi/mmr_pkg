import cascadio
import os
import time

src = "mmr_bot.step"
dst = "tools/mmr_bot.glb"
t0 = time.time()
# tol_linear / tol_angular in the STEP's own units (mm here)
cascadio.step_to_glb(src, dst, tol_linear=0.1, tol_angular=0.3)
print(f"ok {dst} {os.path.getsize(dst)/1e6:.1f} MB in {time.time()-t0:.1f}s")
