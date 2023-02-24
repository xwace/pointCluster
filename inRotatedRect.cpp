float GetCross(Point2f p1, Point2f p2, Point2f p) {
    return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

//判断点p是否在p1p2p3p4的正方形内
int IsPointInMatrix(Point2f p1, Point2f p2, Point2f p3, Point2f p4, Point2f p) {
    int isPointIn = GetCross(p1, p2, p) * GetCross(p3, p4, p) >= 0 && GetCross(p2, p3, p) * GetCross(p4, p1, p) >= 0;
    return isPointIn;
}
