function path = findPath(tree, coor)
    % 从树中找到从起点到指定点的路径
    index = find(tree.x == coor(1) & tree.y == coor(2) & tree.z == coor(3));
    path = [];
    while index ~= 0
        path = [path; tree.x(index), tree.y(index), tree.z(index)];
        index = tree.pre(index);
    end
end