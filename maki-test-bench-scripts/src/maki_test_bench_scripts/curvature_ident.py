import cv2
import numpy as np
import argparse
import math

def detect_red_centroids(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0,70,50])
    upper1 = np.array([10,255,255])
    lower2 = np.array([170,70,50])
    upper2 = np.array([180,255,255])
    m1 = cv2.inRange(hsv, lower1, upper1)
    m2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(m1, m2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts = []
    for c in cnts:
        M = cv2.moments(c)
        if M["m00"]!=0:
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            pts.append((cx,cy))
    return pts, mask

def order_points_greedy(pts):
    if not pts:
        return []
    pts = pts.copy()
    pts = sorted(pts, key=lambda p: (p[0],p[1]))
    ordered = [pts.pop(0)]
    while pts:
        last = ordered[-1]
        dists = [((p[0]-last[0])**2+(p[1]-last[1])**2, i) for i,p in enumerate(pts)]
        _, idx = min(dists)
        ordered.append(pts.pop(idx))
    return ordered

def is_closed(ordered):
    if len(ordered)<3:
        return False
    dists = [math.hypot(ordered[i+1][0]-ordered[i][0], ordered[i+1][1]-ordered[i][1]) for i in range(len(ordered)-1)]
    mean_edge = sum(dists)/len(dists) if dists else 0
    last_dist = math.hypot(ordered[0][0]-ordered[-1][0], ordered[0][1]-ordered[-1][1])
    return last_dist < 1.2*mean_edge

def curvature_list(ordered):
    n = len(ordered)
    if n<3:
        return [None]*n
    closed = is_closed(ordered)
    pts = ordered[:]
    if closed:
        pts = ordered + [ordered[0], ordered[1]]
    else:
        pts = [ordered[0]] + ordered + [ordered[-1]]
    curv = []
    for i in range(1, len(pts)-1):
        p_prev = np.array(pts[i-1], dtype=float)
        p = np.array(pts[i], dtype=float)
        p_next = np.array(pts[i+1], dtype=float)
        v1 = p - p_prev
        v2 = p_next - p
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1==0 or n2==0:
            ang = 0.0
            k = 0.0
        else:
            cross = v1[0]*v2[1]-v1[1]*v2[0]
            dot = v1.dot(v2)
            ang = math.atan2(cross, dot)
            k = ang / ((n1+n2)/2)
        curv.append(k)
    if closed:
        return curv[:-2] if len(curv)>2 else curv
    else:
        return curv[1:-1] if len(curv)>2 else curv

def draw_results(img, ordered, curv):
    for i,p in enumerate(ordered):
        cv2.circle(img, p, 6, (0,255,0), -1)
    for i,(p,k) in enumerate(zip(ordered,curv)):
        if k is None:
            txt = "nan"
        else:
            txt = f"{k:.4f}"
        cv2.putText(img, txt, (p[0]+8,p[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
    for i in range(len(ordered)-1):
        cv2.line(img, ordered[i], ordered[i+1], (255,0,0), 1)
    if is_closed(ordered):
        cv2.line(img, ordered[-1], ordered[0], (255,0,0), 1)
    return img

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    img = cv2.imread(args.image)
    pts, mask = detect_red_centroids(img)
    ordered = order_points_greedy(pts)
    curv = curvature_list(ordered)
    for i,k in enumerate(curv):
        print(i, None if k is None else float(k))
    out = draw_results(img.copy(), ordered, curv)
    cv2.imwrite("curvature_result.png", out)
    if args.show:
        cv2.imshow("result", out)
        cv2.imshow("mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__=="__main__":
    main()
