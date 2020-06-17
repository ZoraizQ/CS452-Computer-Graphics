
data Tree a = Nil | Node (Tree a) a (Tree a)
treesum :: Num(a) => Tree a -> a
treesum = \tree ->
    case tree of
        Nil -> 0
        (Node left x right) -> x + treesum left + treesum right
        _ -> error "Invalid type for tree"


main = putStr $ show $ treesum (Node (Node Nil 1 Nil) 2 (Node Nil 3.5 Nil))