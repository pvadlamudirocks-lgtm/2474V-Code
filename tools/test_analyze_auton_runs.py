import io
import math
import unittest
from analyze_auton_runs import headings, linear, parse, summarize

class AnalysisTests(unittest.TestCase):
    def test_heading_wrap(self):
        h=headings([359,1])
        self.assertLess(min(h['circular_mean'],360-h['circular_mean']),1e-9)
        self.assertEqual(h['smallest_covering_arc'],2)
        self.assertAlmostEqual(h['circular_stddev'],1,places=3)

    def test_opposite_headings(self):
        self.assertIsNone(headings([0,180])['circular_mean'])

    def test_missing_and_singleton(self):
        self.assertEqual(linear([]),{'n':0})
        self.assertIsNone(linear([1])['sample_stddev'])

    def test_parser_and_error(self):
        header='AUTONCSV,run,event,route,segment,motion,result,end_x,end_y,end_h,target_x,target_y\n'
        data=header+'AUTONCSV,1,SEGMENT_END,"left,side",B1,m,PASS,3,4,359,0,0\n'
        data+=header+'AUTONCSV,1,SEGMENT_END,"left,side",B1,m,FAIL,0,0,1,nan,nan\n'
        rows=list(parse(io.StringIO(data),'synthetic'))
        out=summarize(rows)
        self.assertEqual(out['runs'],2)
        self.assertEqual(out['success'],1)
        self.assertEqual(out['failure'],1)
        self.assertEqual(out['euclidean_error']['mean'],5)
        self.assertEqual(rows[0]['route'],'left,side')

    def test_unverified_not_success(self):
        out=summarize([{'_run':('test',1,1),'result':'RETURNED_UNVERIFIED','end_h':'nan'}])
        self.assertEqual(out['success'],0)
        self.assertEqual(out['unverified'],1)
        self.assertEqual(out['final_heading']['n'],0)

if __name__=='__main__': unittest.main()
